#!/bin/sh
#This script starts the driving tasks
echo Start the driving tasks

CURR_WD=$PWD
PKILLS=$CURR_WD'/.driving.txt'
if [ -f "$PKILLS" ]
    then
       rm $PKILLS
fi

echo "Start Init Drive Server"
rosrun C51_CarOperation DRC_Vehicle_Init_server.py &
C51_INIT_PID=$!
echo $C51_INIT_PID
sleep 1

echo "Start Driving Server"
rosrun C51_CarOperation DRC_Vehicle_Drive_server.py &
C51_DRIVE_PID=$!
echo $C51_DRIVE_PID
sleep 1

echo "Start Finish Drive Server"
rosrun C51_CarOperation DRC_Vehicle_Finish_server.py &
C51_FINISH_PID=$!
echo $C51_FINISH_PID
sleep 2

echo "Put the atlas in the car"
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{}'
sleep 5

echo "Start GripGearStick"
rosrun C67_CarManipulation GripGearStick &
C67_GSTICK_PID=$!
echo $C67_GSTICK_PID
sleep 2

echo "Start GripHandBrake"
rosrun C67_CarManipulation GripHandBrake &
C67_GHB_PID=$!
echo $C67_GHB_PID
sleep 2

echo "Start GripSteeringWheel"
rosrun C67_CarManipulation GripSteeringWheel &
C67_GSW_PID=$!
echo $C67_GSW_PID
sleep 2

echo "Start CalbrateWheel"
rosrun C67_CarManipulation CalibrateWheel &
C67_CLW_PID=$!
echo $C67_CLW_PID
sleep 2

echo "Start PlaceFeetOnPedals"
rosrun C67_CarManipulation PlaceFeetOnPedals &
C67_PFP_PID=$!
echo $C67_PFP_PID
sleep 2


echo "Start CalibratePedals"
rosrun C67_CarManipulation CalibratePedals &
C67_CLP_PID=$!
echo $C67_CLP_PID
sleep 2

echo "Entering and exiting the car"
rosrun C46_MountVehicle C46_MountVehicle.py &
C46=$!
echo $C46
sleep 2
rosrun C47_DismountVehicle C47_DismountVehicle.py &
C47=$!
echo $C47


echo $C51_INIT_PID $C51_DRIVE_PID $C51_FINISH_PID $C67_GSTICK_PID $C67_GHB_PID $C67_GSW_PID $C67_CLW_PID $C67_PFP_PID $C67_CLP_PID $C46 $C47 >> $PKILLS

exit 0
