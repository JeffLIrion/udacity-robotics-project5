#!/bin/bash

SCRIPT_ROOT_DIRECTORY="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Based on https://github.com/udacity/RoboND-SLAMLAb/blob/master/SLAM.sh
xterm  -e "cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash ; roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$SCRIPT_ROOT_DIRECTORY/../map/worlds/project4b.world " &
sleep 5
#xterm  -e "cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash ; roslaunch turtlebot_teleop keyboard_teleop.launch " &
#sleep 2
#xterm -e "cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash ; rosrun gmapping slam_gmapping  " &
xterm -e "cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash ; roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$SCRIPT_ROOT_DIRECTORY/../map/map.yaml" &
sleep 3
#xterm  -e "cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash ; rosrun rviz rviz -d $SCRIPT_ROOT_DIRECTORY/../rvizConfig/slam.rviz" &
xterm  -e "cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash ; roslaunch launchfiles project5_rviz.launch" &

(cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash; while ! rostopic info /map | grep -q map_server ; do echo 'Waiting for map_server' && sleep 1; done;)
sleep 5
xterm -e "cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash; rosrun add_markers add_markers" &

sleep 15
xterm -e "cd $SCRIPT_ROOT_DIRECTORY/../..; source devel/setup.bash; rosrun pick_objects pick_objects" &
