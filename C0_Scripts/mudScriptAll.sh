#!/bin/sh


###########################
# START BIN
###########################
#killall executer
#sleep 3	
#echo "START"
#rosservice call /executer/run "T1" "$HOME/git/robil/C34_Designer/plans/FollowPath.xml"
#rosservice call /executer/resume "T1"
#sleep 10
#echo "LOADED PLAN"
#cd /$HOME/git/robil/C42_DynamicLocomotion/scripts/
#python ExitStartingPen.py &
#echo "LOADED POINTS"
#PID=$!
#RUNNING="true"
#FLAG=1
#while [ $FLAG -gt 0 ]; do 
#	TEST=`rosservice call /executer/stack T1`
#	if `echo $TEST | grep "T1 is finished" 1>/dev/null 2>&1`
#	then
#		FLAG=0
#		kill $PID
#		echo "DONE"	  
#	else
#	  echo "RUNNING ExitStartingPen"
#	fi
#	sleep 3	
#done

###########################
# GATE2
###########################
killall executer
sleep 3	
echo "START"
rosservice call /executer/run "T2" "$HOME/git/robil/C34_Designer/plans/FollowPath.xml"
rosservice call /executer/resume "T2"
sleep 10
echo "LOADED PLAN"
cd /$HOME/git/robil/C42_DynamicLocomotion/scripts/
python Task2Gate2.py &
echo "LOADED POINTS"
PID=$!
FLAG=1
while [ $FLAG -gt 0 ]; do 
	TEST=`rosservice call /executer/stack T2`
	if `echo $TEST | grep "T2 is finished" 1>/dev/null 2>&1`
	then
		FLAG=0
		kill $PID
		echo "DONE"	  
	else
	  echo "RUNNING Task2Gate2"
	fi
	sleep 3	
done

###########################
# MUD
###########################
#rosrun C42_State state &
#sleep 3
#rosrun C43_LocalBodyCOM computeCOM &
#sleep 3
killall executer
sleep 3	
echo "START"
rosservice call /executer/run "T3" "$HOME/git/robil/C34_Designer/plans/Mud.xml"
rosservice call /executer/resume "T3"
sleep 10
echo "LOADED PLAN"
cd /$HOME/git/robil/C42_DynamicLocomotion/scripts/
python MudPit.py &
echo "LOADED POINTS"
PID=$!
FLAG=1
while [ $FLAG -gt 0 ]; do 
	TEST=`rosservice call /executer/stack T3`
	if `echo $TEST | grep "T3 is finished" 1>/dev/null 2>&1`
	then
		FLAG=0
		kill $PID
		echo "DONE"	  
	else
	  echo "RUNNING MudPit"
	fi
	sleep 3	
done

###########################
# HILS
###########################
killall executer
sleep 3	
echo "START"
rosservice call /executer/run "T4" "$HOME/git/robil/C34_Designer/plans/HillsBwd.xml"
rosservice call /executer/resume "T4"
sleep 10
echo "LOADED PLAN"
cd /$HOME/git/robil/C42_DynamicLocomotion/scripts/
python TerrainHills.py &
echo "LOADED POINTS"
PID=$!
FLAG=1
while [ $FLAG -gt 0 ]; do 
	TEST=`rosservice call /executer/stack T1`
	if `echo $TEST | grep "T4 is finished" 1>/dev/null 2>&1`
	then
		FLAG=false
		kill $PID
		echo "DONE"	  
	else
	  echo "RUNNING TerrainHills"
	fi
	sleep 3	
done

###########################
# DEBRIS
###########################
killall executer
sleep 3	
echo "START"
rosservice call /executer/run "T5" "$HOME/git/robil/C34_Designer/plans/HillsBwd.xml"
rosservice call /executer/resume "T5"
sleep 10
echo "LOADED PLAN"
cd /$HOME/git/robil/C42_DynamicLocomotion/scripts/
python TerrainsDebris.py &
echo "LOADED POINTS"
PID=$!
FLAG=1
while [ $FLAG -gt 0 ]; do 
	TEST=`rosservice call /executer/stack T5`
	if `echo $TEST | grep "T5 is finished" 1>/dev/null 2>&1`
	then
		FLAG=false
		kill $PID
		echo "DONE"	  
	else
	  echo "RUNNING"
	fi
	sleep 3	
done
