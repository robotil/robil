#!/bin/bash
rostopic pub --once /drc_world/robot_enter_car geometry_msgs/Pose "position:
  x: 0.0
  y: 0.0
  z: -0.1
orientation:
  x: 0.0
  y: 0.0
  z: 0.0
  w: 0.0"

rostopic pub --once /multisense_sl/set_spindle_speed std_msgs/Float64 "data: 6.0"


