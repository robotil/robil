# This message contains information of which foot to place and the position and orientation 
# where to place the foot. Also includes the foot clearance height of the foots swing trajectory.   
uint32     foot_index         # Foot_index can be LEFT_FOOT=0 or RIGHT_FOOT=1
Pose_Euler pose               # Foot pose in world frame
float64    clearance_height   # Step apex swing height measured form the
                              # midpoint between the feet
