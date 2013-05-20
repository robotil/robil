#!/bin/bash
rostopic pub --once /multisense_sl/set_spindle_speed std_msgs/Float64 "data: 1.0"
