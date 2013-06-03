#!/bin/sh
#This script should kill all the running modules

CURR_WD=$PWD
PKILLF=$CURR_WD'/.c11agent.txt'
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

