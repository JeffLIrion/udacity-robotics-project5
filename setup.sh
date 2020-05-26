#!/bin/bash

SCRIPT_ROOT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src"

# catkin_init_workspace
cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src"
catkin_init_workspace

# catkin_make
cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
catkin_make
source devel/setup.bash

# clone repos
cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src"
git clone https://github.com/ros-perception/slam_gmapping.git
git clone https://github.com/turtlebot/turtlebot.git
git clone https://github.com/turtlebot/turtlebot_interactions.git
git clone https://github.com/turtlebot/turtlebot_simulator.git


cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
rosdep -i install slam_gmapping turtlebot turtlebot_interactions turtlebot_simulator


# catkin_make
cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
catkin_make
source devel/setup.bash


# Create directories for packages that I will create

# Inside this directory, you will store your gazebo world file and the map generated from SLAM.
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/map"

# Inside this directory, you’ll store your shell scripts.
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/scripts"

# Inside this directory, you’ll store your customized rviz configuration files.
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/rvizConfig"

# You will write a node that commands your robot to drive to the pickup and drop off zones.
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/pick_objects"

#  You will write a node that model the object with a marker in rviz.
mkdir -p "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src/add_markers"


# Create and build a package of launch files
cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws/src"
catkin_create_pkg launchfiles
cd "$SCRIPT_ROOT_DIRECTORY/catkin_ws"
catkin_make
