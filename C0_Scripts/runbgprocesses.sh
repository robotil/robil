#!/bin/sh
#This script should start the perception modules

CURR_WD=$PWD
PKILL=$CURR_WD'/.killme.txt'
rm $PKILL
echo "Starting C46 Task"
rosrun C46_MountVehicle C46_MountVehicle.py &
C46_PID=$!
echo $C46_PID
sleep 1
echo "Starting C47"
rosrun C47_DismountVehicle  C47_DismountVehicle.py &
C47_PID=$!
echo $C47_PID
sleep 1
echo "Starting C48"
rosrun C48_StandUp C48_StandUp.py &
C48_PID=$!
echo $C48_PID
sleep 1
echo "Starting C51 Init"
rosrun C51_CarOperation DRC_Vehicle_Init_server.py &
C51_Init_PID=$!
echo $C51_Init_PID
sleep 1
echo "Starting C51 Waypoint Driving"
rosrun C51_CarOperation DRC_Vehicle_Drive_server.py &
C51_Drive_PID=$!
echo $C51_Drive_PID
sleep 1
echo "Starting C51 Finish"
rosrun C51_CarOperation DRC_Vehicle_Finish_server.py &
C51_Finish_PID=$!
echo $C51_Finish_PID
echo $C46_PID $C47_PID $C48_PID $C51_Init_PID $C51_Drive_PID $C51_Finish_PID>> $PKILL
echo "Done "
