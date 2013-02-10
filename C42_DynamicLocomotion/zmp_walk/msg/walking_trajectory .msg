# This message contains information of desired limb positions and orientation
# to be used by Inverse Kinematics function.
# The positions and orientations are relative to "forward stance foot".
# => stance_foot (x,y,z,r,p,w)=(0,0,0,0,0,0)
# x-forward, y-left side of robot, z-up, r-roll around x, p-pitch around y, w-yaw around z
# Units are [meter] and [rad]. 

# Stance leg information:
Position      stance_hip    # desired stance hip position (x,y,z) usage e.g: stance_hip.x
Orientation   pelvis_d      # desired pelvis orientation (r,p,w), to adjust robots posture 

# Swing leg information:
Pos_and_Ori   swing_foot    # desired swing foot position and orientation (x,y,z,r,p,w), 
                            # when swing foot is on the ground changes to measured (x,y,z,r,p,w).
Position      swing_hip     # desired swing hip position (x,y,z) 
Orientation   pelvis_m      # measured pelvis orientation(r,p,w) 

# COM information:
Position      com_ref       # desired COM position (x,y,z)
Position      com_dot_ref   # desired COM velocity (x,y,z) [m/sec]
Position      com_m         # measured COM position (x,y,z)

# Step phase information:
int32        step_phase     # 1 = Double-Support left leg in front, 2 = lift right swing leg
                            # 3 = Double-Support right leg in front, 4 = lift left swing leg 

