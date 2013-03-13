#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
PKILL=$CURR_WD'/.killwithpath.txt'
rm $PKILL
#roslaunch atlas_utils atlas_golf_cart_fire_hose.launch
echo "Starting C31"
rosrun C31_PathPlanner gpp&
C31_PID=$!
echo $C31_PID
echo "Done "
echo $C31_PID  >> $PKILL
cd $CURR_WD

rosservice call executer/run T /home/michele/git/robil/C0_Scripts/pln
rosservice call executer/resume T
rosservice call executer/run T1 /home/michele/git/robil/C0_Scripts/pln1
rosservice call executer/resume T1
