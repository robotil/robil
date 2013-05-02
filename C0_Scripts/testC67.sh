#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
PKILL=$CURR_WD'/.killC67test.txt'
rm $PKILL
echo "This is a self test for the driving dexterity tasks"
echo "Setting the robot in the driver seat"
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{}'
echo "Done"
echo "Grip Steering Wheel"
rosrun C67_CarManipulation GripSteeringWheel &
C1=$!
sleep 1
echo "Place Feet on Pedals"
rosrun C67_CarManipulation PlaceFeetOnPedals &
C2=$!
sleep 1
echo "Calibrate Wheel"
rosrun C67_CarManipulation CalibrateWheel &
C3=$!
sleep 1
echo "Calibrate Pedals"
rosrun C67_CarManipulation CalibratePedals &
C4=$!
sleep 1
echo $C1 $C2 $C3 $C4  >> $PKILL

