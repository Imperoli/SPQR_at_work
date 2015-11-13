#!/bin/bash
xterm -e roscore &
sleep 3
xterm -e roslaunch openni_launch openni.launch &
sleep 3
xterm -e rosrun rqt_reconfigure rqt_reconfigure &
sleep 3
xterm -e rosrun rviz rviz &
sleep 3
xterm -e rosrun object_recognition_core detection -c `rospack find object_recognition_tabletop`/conf/detection.table.ros.ork &
sleep 3
xterm -e rosrun obj_detect obj_detect_node &