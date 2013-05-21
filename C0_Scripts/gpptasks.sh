#!/bin/sh

rosservice call executer/run T $ROBIL/C0_Scripts/pln
rosservice call executer/resume T
rosservice call executer/run T1 $ROBIL/C0_Scripts/pln1
rosservice call executer/resume T1
