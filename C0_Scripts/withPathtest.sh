#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
PKILL=$CURR_WD'/.killwithpath.txt'
rm $PKILL
#roslaunch atlas_utils atlas_golf_cart_fire_hose.launch
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
echo "Starting C31"
rosrun C31_PathPlanner gpp&
C31_PID=$!
echo $C31_PID
echo "Done "
echo $C31_PID  >> $PKILL
#echo $C22_PID $C31_PID $C34_PID >> $PKILL
cd $CURR_WD

rosservice call executer/run T /userhome/mhallak/git/robil/C0_Scripts/pln
rosservice call executer/resume T
rosservice call executer/run T1 /userhome/mhallak/git/robil/C0_Scripts/pln1
rosservice call executer/resume T1
