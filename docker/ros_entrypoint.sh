#!/bin/bash
set -e

# setup ros environment
#source "/opt/ros/$ROS_DISTRO/setup.bash"

source /catkin_ws/devel/setup.bash && roscore &> /dev/null &

alias ws2='/catkin_ws/devel/setup.bash'

exec "$@"
#source "/catkin_ws/devel/setup.bash" 
