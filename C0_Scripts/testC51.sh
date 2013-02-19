#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
PKILL=$CURR_WD'/.killC51test.txt'
rm $PKILL
echo "This is a self test for the driving tasks"
#roslaunch C51_CarOperation C51_completeLuanch.launch &
#C51_PID=$!
#echo $C51_PID
#echo "sleeping 60"
#sleep 60
echo "Setting the robot in the driver seat"
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{}'
echo "Done"
echo "Initializing the car"
rosrun C51_CarOperation DRC_Vehicle_Init_client.py &
INIT=$!
echo $INIT
sleep 3
echo "Driving the car"
rosrun C51_CarOperation DRC_Vehicle_Drive_client.py &
DRIVE=$!
echo $DRIVE
echo "Done "
sleep 1
echo "Finishing driving"
rosrun C51_CarOperation DRC_Vehicle_Finish_client.py &
FIN=$!
echo $FIN
echo "Done "
echo  $INIT $DRIVE $FIN >> $PKILL
#echo $C51_PID $INIT $DRIVE $FIN >> $PKILL

