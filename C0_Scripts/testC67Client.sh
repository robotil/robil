#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
echo "This is a self test for the driving dexterity tasks"
echo "Steering Wheel"
rosrun C67_CarManipulation SteeringWheel_client
sleep 1
echo "Brake Pedal"
rosrun C67_CarManipulation BrakePedal_client 
sleep 1
echo "Gas Pedal"
rosrun C67_CarManipulation GasPedal_client 
sleep 1
echo "Gear Handle"
rosrun C67_CarManipulation GearHandle_client
sleep 1
echo "Hand Brake"
rosrun C67_CarManipulation HandBrake_client
sleep 1
echo "Ignition"
rosrun C67_CarManipulation Ignition_client 

