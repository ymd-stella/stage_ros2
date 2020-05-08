#pragma once
#include <mutex>
#include <stage.hh>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <ranger_wrapper.hpp>
#include <camera_wrapper.hpp>
#include <position_wrapper.hpp>

namespace {
void replaceAll(std::string& str, const std::string& from, const std::string& to) {
    if(from.empty())
        return;
    size_t start_pos = 0;
    while((start_pos = str.find(from, start_pos)) != std::string::npos) {
        str.replace(start_pos, from.length(), to);
        start_pos += to.length();
    }
}
}

class RobotWrapper
{
public:
    RobotWrapper(const rclcpp::executors::SingleThreadedExecutor::SharedPtr& executor, std::string name) {
        tf_prefix_ = name;
        node_ = rclcpp::Node::make_shared(name, "stage_ros2");
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            "~/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(1)),
            std::bind(&RobotWrapper::cmd_vel_callback, this, std::placeholders::_1));
        executor->add_node(node_);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        position_->set_speed(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void publish(const rclcpp::Time& now) {
        geometry_msgs::msg::TransformStamped transform;
        std::string frame_id = "base_footprint";
        std::string child_frame_id = "base_link";
        if (tf_prefix_.size() > 0) {
            frame_id = tf_prefix_ + "/" + frame_id;
            child_frame_id = tf_prefix_ + "/" + child_frame_id;
        }
        transform.header.frame_id = frame_id;
        transform.header.stamp = now;
        transform.child_frame_id = child_frame_id;
        tf_broadcaster_->sendTransform(transform);

        {
            std::lock_guard<std::mutex> lock(mtx_);
            position_->publish(tf_broadcaster_, now);
            for (auto& ranger : rangers_) {
                ranger->publish(tf_broadcaster_, now);
            }
            for (auto& camera : cameras_) {
                camera->publish(tf_broadcaster_, now);
            }
        }
    }

    void wrap(Stg::Model* mod) {
        if (mod->GetModelType() == "position") {
            mod->Subscribe();
            position_ = std::make_shared<PositionWrapper>(node_, static_cast<Stg::ModelPosition*>(mod), tf_prefix_);
        }
        else {
            // token -> model_name
            std::string model_name = mod->Token();
            auto pos_dot = model_name.find(".");
            if (pos_dot != std::string::npos) {
                model_name = model_name.substr(pos_dot + 1);
            }
            replaceAll(model_name, ":", "_");

            if (mod->GetModelType() == "ranger") {
                mod->Subscribe();
                rangers_.push_back(std::make_shared<RangerWrapper>(node_, static_cast<Stg::ModelRanger*>(mod), model_name, tf_prefix_));
            }
            else if (mod->GetModelType() == "camera") {
                mod->Subscribe();
                cameras_.push_back(std::make_shared<CameraWrapper>(node_, static_cast<Stg::ModelCamera*>(mod), model_name, tf_prefix_));
            }
            else {
                RCLCPP_WARN_STREAM(node_->get_logger(), "sensor " << mod->GetModelType() << "is not supported");
            }
        }
    }

    std::mutex mtx_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::string tf_prefix_;
    std::shared_ptr<PositionWrapper> position_;
    std::vector<std::shared_ptr<CameraWrapper>> cameras_;
    std::vector<std::shared_ptr<RangerWrapper>> rangers_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
};