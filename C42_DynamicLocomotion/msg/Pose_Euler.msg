# A representation of pose in free space, composed of postion and orientation according to euler angles. 
geometry_msgs/Point    position   # cartesian position [meters] (x,y,z), example: position.x 
geometry_msgs/Vector3  ang_euler  # Euler angles [rad] (x,y,z)<=>(roll - around x, pitch - around y, yaw - around z),
                                  # example: ang_euler.x accesses the roll angle
