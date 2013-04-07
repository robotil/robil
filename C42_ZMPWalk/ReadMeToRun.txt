TO RUN STUB:
- run ZMP_stub.py: 
   rosrun C42_ZMPWalk ZMP_stub.py

- run C31_clone to emulate the C31PathPlanner (publish a path message): 
   rosrun C42_ZMPWalk scripts/C31_Clone.py

- execute using C0 task_tester: 
   rosrun C0_RobilTask task_tester FollowPath

