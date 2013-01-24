#!/bin/sh
#This script should be ran from within the
#scripts folder

CURR_WD=$PWD
LOG=$CURR_WD'/log.txt'
rm $LOG
echo 'Logging to' $LOG
date >> $LOG
cd ..
ROS_WD=$PWD
export ROS_PACKAGE_PATH=$ROS_WD:$ROS_PACKAGE_PATH

dir="$ROS_WD/C34_BTExecuter"
if test -d "$dir"; then
        echo ">> $dir" 
        cd $dir/bin_so
        make all 2>&1 2>&1 2>&1 >/dev/null
        STATUS=$?
        if [ $STATUS -ne 0 ]; then
                                echo "[Error] $dir" >> $LOG
        else
                echo "[PASS] $dir" >> $LOG
        fi
        echo "<< $dir"
        cd $ROS_WD
fi

for dir in "$ROS_WD"/*; do
	if test -d "$dir"; then
                bn=` basename $dir`
                echo "bn=$bn"
                if [ "$bn" != "C0_Scripts" ]; then
                                if [ "$bn" != "launch" ]; then
					echo ">> $dir" 
					cd $dir
					#rosmake --pjobs=4;
					rosmake;
					# --pjobs=8 --threads=4;
					STATUS=$?
					if [ $STATUS -ne 0 ]; then
       					 	echo "[Error] $dir" >> $LOG
   					else
						echo "[PASS] $dir" >> $LOG
					fi 
					echo "<< $dir"
					cd $ROS_WD
                                fi
                fi
	fi
done
date >> $LOG
cd $CURR_WD 

exit 0
