#!/bin/sh
#This script should start the anything related to OCU at the site 

CURR_WD=$PWD
PKILL=$CURR_WD'/.c11agent.txt'
rm $PKILL

eco "TimeoutMonitor"
python $ROBIL/C35_Monitoring/src/TimeoutMonitor.py &
C2=$!
sleep 1

echo "Running C11_Agent "
rosrun C11_Agent C11_Agent &
C11=$!  

echo "Running GPP"
rosrun C31_PathPlanner gpp | tee gpp.log  &
C31=$!

echo "Running Executer "
rosrun C34_Executer executer &
C34=$!  

echo $C11 $C31 $C34 >> $PKILL

exit 0
