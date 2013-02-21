#!/bin/sh
#This script should run the modules needed for testing object recognition

CURR_WD=$PWD
PKILL=$CURR_WD'/.killwithobjectrec.txt'
rm $PKILL
echo "Starting C23"
rosrun C23_ObjectRecognition c23 /multisense_sl/camera/left/image_color /multisense_sl/camera/right/image_color &
C23_PID=$!
echo $C23_PID
echo "Done "
echo $C23_PID  >> $PKILL
#echo $C22_PID $C31_PID $C34_PID >> $PKILL

rosservice call executer/run O /home/userws4/git/robil/C34_Designer/plans/trackObject.xml
rosservice call executer/resume O
rosservice call executer/run S /home/userws4/git/robil/C34_Designer/plans/searchObject.xml
rosservice call executer/resume S
