#!/bin/sh
#This script should be ran from within the
#scripts folder

CURR_WD=$PWD
rm log.txt
cd ..
ROS_WD=$PWD
for dir in "$ROS_WD"/*; do
	if test -d "$dir"; then
		echo ">> $dir" 
		cd $dir
		make clean;
		echo "<< $dir"
		cd ..
	fi
done
cd $CURR_WD 

exit 0
