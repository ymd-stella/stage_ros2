cmake_minimum_required(VERSION 3.5)
project(stage_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stage REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(stage REQUIRED)
add_executable(stage_ros2 src/stage_ros2.cc)
# build
target_include_directories(stage_ros2
    PRIVATE ${STAGE_INCLUDE_DIRS} include)
target_link_libraries(stage_ros2
    PRIVATE ${STAGE_LIBRARIES})
ament_target_dependencies(stage_ros2
    PUBLIC 
    rclcpp
    rcutils
    cv_bridge
    image_transport
    geometry_msgs
    nav_msgs
    sensor_msgs
    stage
    std_msgs
    std_srvs
    tf2
    tf2_ros
    tf2_geometry_msgs)
install(TARGETS stage_ros2
    DESTINATION lib/${PROJECT_NAME})

ament_package()