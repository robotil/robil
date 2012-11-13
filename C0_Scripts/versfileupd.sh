#!/bin/sh
#This script should be ran from within the
#scripts folder

CURR_WD=$PWD
VER=$CURR_WD'/../version'
FILE='.ver'
rm $VER
cd ..
ROS_WD=$PWD
for dir in "$ROS_WD"/*; do
	if test -d "$dir"; then
		cd $dir
		echo -n ` basename $dir ` >> $VER
		echo -n "     " >> $VER
		if [ -e $FILE ]; then cat $FILE >> $VER; 
		else echo "file $FILE in $dir doesn't exist.";
		fi
		cd ..
	fi
done
cd $CURR_WD 

exit 0
