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

dir="$ROS_WD/C34_BTExecuter"
if test -d "$dir"; then
        echo ">> $dir" 
        cd $dir/bin_so
        make all &>/dev/null
        STATUS=$?
        if [ $STATUS -ne 0 ]; then
                echo "[Error] $dir" >> $LOG
        else
                echo "[PASS] $dir" >> $LOG
                cd ../bin_exe
                make all &>/dev/null && cp BTExecuter ../../C34_Designer/BTExecuter-lin.bin
        fi
        echo "<< $dir"
        cd $ROS_WD
fi

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
		cd $ROS_WD
	fi
done

cd $CURR_WD 

exit 0
