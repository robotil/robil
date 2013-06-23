#!/bin/sh
#This script should start the posture modules

CURR_WD=$PWD
PKILL=$CURR_WD'/.pc.txt'
rm $PKILL

#echo "Starting DRCSim tools"
#rosrun C42_DRCSim2_tools foot_contact_filter.py &
#C42T_PID=$!
#echo "Starting C45"
roslaunch C45_PostureControl C45_PostureControl.launch &
C45_PID=$!
sleep 4
echo $C45_PID >> $PKILL
#echo $C25_PID $C42T_PID $C45_PID >> $PKILL
#echo "Start  PoseController"
rosservice call /PoseController/start "{}" 
echo "Started"
echo "Move head position 0: rosservice call /PoseController/back_lbz_neck_ay 'back_lbz: 0.0 
neck_ay: 0.0'"
rosservice call /PoseController/back_lbz_neck_ay "back_lbz: 0.0
neck_ay: 0.0"

echo "Done"
rosservice call /PoseController/stop "{}" 
echo "PoseController stopped"
kill $C45_PID
