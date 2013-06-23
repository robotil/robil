#!/bin/bash

python $PWD/move_AT_neck.py

rosservice call /PoseController/start '{}'

rosservice call /PoseController/neck_movement "neck_ay: $1"

rosservice call /PoseController/stop '{}'
