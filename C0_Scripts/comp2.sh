#!/bin/sh
#This script should start the comp2 modules

echo "Perception...."
./perception.sh
sleep 3
echo "Standup...."
./standup.sh
sleep 3
echo "Posture Control and Stability"
./postCont.sh
sleep 4
echo "Walking"
./walking.sh
sleep 1
echo "C11 Agent"
./c11Agent.sh

exit 0
