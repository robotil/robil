#!/bin/sh
#This script should run the modules needed for standup functionnality

CURR_WD=$PWD
PKILL=$CURR_WD'/.killStandUp.txt'
rm $PKILL
echo "This script run some tasks needed for standup"
echo "Checking IMU"
rosrun C25_AtlasFell Fell_server.py &
C01=$!
rosrun C25_AtlasFell Fell_client.py &
C02=$!
sleep 2
echo "FallingMonitor"
rosrun C35_Monitoring FallingMonitor.py &
C1=$!
sleep 1
#echo "TimeoutMonitor"
#python $ROBIL/C35_Monitoring/src/TimeoutMonitor.py &
#C2=$!
#sleep 1
echo "Standup"
rosrun C48_StandUp C48_StandUp.py &
C3=$!
sleep 1
echo $C01 $C02 $C1 $C3  >> $PKILL
#echo $C01 $C02 $C1 $C2 $C3  >> $PKILL
#echo $C1 $C2 $C3 $C4 >> $PKILL

