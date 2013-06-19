#!/bin/sh
#This script should start the posture modules

echo "Start  PoseController"
rosservice call /PoseController/start "{}" 
echo "Started"
#rosservice call /PoseController/neck_movement "neck_ay: 0.0"
echo "Move head position 0: rosservice call /PoseController/back_lbz_neck_ay 'back_lbz: 0.0 
neck_ay: 0.7'"
rosservice call /PoseController/back_lbz_neck_ay "back_lbz: 0.0
neck_ay: 0.7"

echo "Done"
echo "Left hand position 0"
rosservice call /lPath_srv "PositionDestination:
  x: -0.128
  y: 0.414
  z: 0.07
AngleDestination:
  x:  2.749
  y: 0.76378
  z: 1.408
time: 3.0
points: 100.0"
echo "Done" 1
rosservice call /lPath_srv "PositionDestination:
  x: -0.389
  y: 0.569
  z: 0.0
AngleDestination:
  x: 2.803
  y: 0.927
  z: 1.471
time: 3.0
points: 100.0"
echo "Done 2"
rosservice call /lPath_srv "PositionDestination:
  x: -0.389
  y: 0.582
  z: 0.552
AngleDestination:
  x: 2.734
  y: -0.862
  z: 2.309
time: 3.0
points: 100.0"
echo "Done 3"
rosservice call /lPath_srv "PositionDestination:
  x: 0.3
  y: 0.4
  z: 0.4
AngleDestination:
  x: 2.762
  y: 0.5
  z: 0.0
time: 3.0
points: 100.0"
echo "Done 4"
rosservice call /lPath_srv "PositionDestination:
  x: 0.55
  y: 0.071
  z: 0.42
AngleDestination:
  x: -3.14
  y: 0.0
  z: -1.442
time: 3.0
points: 100.0" 
echo "Done 5"

rostopic pub /sandia_hands/l_hand/simple_grasp sandia_hand_msgs/SimpleGrasp "name: 'cylindrical' 
closed_amount: 0.5"
echo "Done 6"
#rosservice call /PoseController/stop "{}" 
#echo "PoseController stopped"
#kill $C45_PID
