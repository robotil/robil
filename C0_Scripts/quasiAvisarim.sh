#!/bin/sh
#This script should run the modules needed for testing tasks needing path

CURR_WD=$PWD
PKILL=$CURR_WD'/.killQuasiAvizarim.txt'
rm $PKILL
echo "C42_DRCSim2_tools"
python $ROBIL/C42_DRCSim2_tools/src/foot_contact_filter.py &
C01=$!
roslaunch C45_PostureControl C45_PostureControl.launch &
C02=$!
sleep 2
echo "C43"
rosrun C43_LocalBodyCOM comteCOM &
C1=$!
sleep 1
roslaunch C41_QuasiStaticWalking C41_QuasiStaticWalking.launch &
C2=$!
sleep 1
echo $C01 $C02 $C1 $C2  >> $PKILL
#echo $C1 $C2 $C3 $C4 >> $PKILL

