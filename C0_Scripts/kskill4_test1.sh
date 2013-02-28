#!/bin/sh
#This script should kill all the running modules

CURR_WD=$PWD
PKILLFS=$CURR_WD'./skill4_test1.txt'
#cat $PKILLFS >> $PKILLFS

./kperception.sh
echo "Perception Killed "
./killObjRec.sh 
echo "Obj Recogn Killed "
#./killWithPath.sh

if test -e $PKILLFS; then

	echo "Killing "
	cat $PKILLFS

	kill `cat $PKILLFS`

	echo "Killed "
else echo "No processes to kill"
fi
#echo $PKILLFS
#kill $PKILLFS

