#!/bin/sh
#This script should start the perception modules

CURR_WD=$PWD
PKILLF=$CURR_WD'/.killme.txt'
#cat $PKILLF >> $PKILLP
if test -e $PKILLF; then

	echo "Killing "
	cat $PKILLF

	kill `cat $PKILLF`

	echo "Killed "
else echo "No processes to kill"
fi
#echo $PKILLP
#kill $PKILLP

