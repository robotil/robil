#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
PKILL=$CURR_WD'/.killC67test.txt'
rm $PKILL
echo "This is a self test for the driving dexterity tasks"
echo "Setting the robot in the driver seat"
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose '{}'
echo "Done"
echo "Steering Wheel"
rosrun C67_CarManipulation SteeringWheel_server  &
C1=$!
sleep 1
echo "Brake Pedal"
rosrun C67_CarManipulation BrakePedal_server  &
C2=$!
sleep 1
echo "Gas Pedal"
rosrun C67_CarManipulation GasPedal_server &
C3=$!
sleep 1
echo "Gear Handle"
rosrun C67_CarManipulation GearHandle_server &
C4=$!
sleep 1
echo "Hand Brake"
rosrun C67_CarManipulation HandBrake_server &
C5=$!
sleep 1
echo "Ignition"
rosrun C67_CarManipulation Ignition_server &
C6=$!
sleep 1
echo $C1 $C2 $C3 $C4 $C5 $C6 >> $PKILL

