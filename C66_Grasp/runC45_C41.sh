#!/bin/bash
rosrun C42_DRCSim2_tools foot_contact_filter.py &
rosrun C43_LocalBodyCOM computeCOM &
roslaunch C45_PostureControl C45_PostureControl.launch &
sleep 1
roslaunch C41_QuasiStaticWalking C41_QuasiStaticWalking.launch &

