# stage_ros2

[![Join the chat at https://gitter.im/stage_ros2/community](https://badges.gitter.im/stage_ros2/community.svg)](https://gitter.im/stage_ros2/community?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

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