#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
PKILL=$CURR_WD'/.killwithobjectrec.txt'
rm $PKILL
#roslaunch atlas_utils atlas_golf_cart_fire_hose.launch
#rostopic pub /drc_world/robot_enter_car geometry_msgs/Pose "position (press TAB)
#echo "Starting executer"
#rosrun C34_Executer executer &
#C34_PID=$!
#echo $C34_PID
#echo "Done "
#sleep 1
#rosrun C22_GroundRecognitionAndMapping C22 &
#C22_PID=$!
#echo $C22_PID
#sleep 3
echo "Starting C23"
rosrun C23_ObjectRecognition c23 /multisense_sl/camera/left/image_color /multisense_sl/camera/right/image_color &
C23_PID=$!
echo $C23_PID
echo "Done "
echo $C23_PID  >> $PKILL
#echo $C22_PID $C31_PID $C34_PID >> $PKILL

rosservice call executer/run O /userhome/mhallak/git/robil/C34_Designer/plans/trackObject.xml
rosservice call executer/resume O
rosservice call executer/run S /userhome/mhallak/git/robil/C34_Designer/plans/searchObject.xml
rosservice call executer/resume S
