#!/bin/bash

export ROS_MASTER_URI=http://jaythink.local:11311

source /opt/ros/melodic/setup.bash
source ~/ros_workspaces/kamigami_ws/devel/setup.bash

exec "$@"
