#!/bin/sh

CURR_WD=$PWD
cd ..
rm -rf `find . -type d -name .svn`
cd $CURR_WD 

exit 0


