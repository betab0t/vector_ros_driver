#!/bin/bash

# exit on error
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.bash"
source "/cv_bridge_build_ws/install/setup.bash" --extend

# configure sdk
/catkin_ws/src/vector_ros_driver/sdk_auto_config.sh

# execute command
exec "$@"
