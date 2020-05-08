# stage_ros2
ROS2 wrapper for the Stage simulator.

# Example
```
cd ros2_ws/src
git clone https://github.com/ymd-stella/stage_ros2.git
cd ..
colcon build --symlink-install
source install/setup.sh
ros2 run stage_ros2 stage_ros2 --ros-args -p world:=camera.world
```