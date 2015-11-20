#!/bin/sh

# Start simulated environment and robot

`(rospack find diago_apps)`/script/run_movebase.py diago DISB1 SIM 

sleep 5

xterm -e "roslaunch hri_pnp hri_pnpas.launch" &

sleep 5

rostopic pub /diago/planToExec std_msgs/String "data: 'go_and_greet_person'"  --once

sleep 3

rostopic pub /diago/PNPConditionEvent std_msgs/String "data: 'personAt_RoomB115'" --once


