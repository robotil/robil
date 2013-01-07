#!/bin/sh

eval "rosrun pr2_controller_manager pr2_controller_manager kill back_lbz_position_controller"
sleep 1
eval "rosrun pr2_controller_manager pr2_controller_manager spawn back_lbz_position_controller"
