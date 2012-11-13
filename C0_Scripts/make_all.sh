#!/bin/sh
#This script should be ran from within the
#scripts folder

CURR_WD=$PWD
LOG=$CURR_WD'/log.txt'
rm $LOG
echo 'Logging to' $LOG
cd ..
ROS_WD=$PWD
export ROS_PACKAGE_PATH=$ROS_WD:$ROS_PACKAGE_PATH
for dir in "$ROS_WD"/*; do
	if test -d "$dir"; then
		echo ">> $dir" 
		cd $dir
		rosmake;
		STATUS=$?
		if [ $STATUS -ne 0 ]; then
       			 echo "[Error] $dir" >> $LOG
   		else
			echo "[PASS] $dir" >> $LOG
		fi 
		echo "<< $dir"
		cd ..
	fi
done
cd $CURR_WD 

exit 0
