#!/bin/sh
#This script should run the BDI switch


echo "Calling BDIswitch state to 0"
rosservice call C25/BDIswitch "state:
   data: 0"
