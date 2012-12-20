; Auto-generated. Do not edit!


(cl:in-package C21_VisionAndLidar-msg)


;//! \htmlinclude C21C0_3DR.msg.html

(cl:defclass <C21C0_3DR> (roslisp-msg-protocol:ros-message)
  ((cloud
    :reader cloud
    :initarg :cloud
    :type sensor_msgs-msg:PointCloud2
    :initform (cl:make-instance 'sensor_msgs-msg:PointCloud2)))
)

(cl:defclass C21C0_3DR (<C21C0_3DR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C21C0_3DR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C21C0_3DR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C21_VisionAndLidar-msg:<C21C0_3DR> is deprecated: use C21_VisionAndLidar-msg:C21C0_3DR instead.")))

(cl:ensure-generic-function 'cloud-val :lambda-list '(m))
(cl:defmethod cloud-val ((m <C21C0_3DR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-msg:cloud-val is deprecated.  Use C21_VisionAndLidar-msg:cloud instead.")
  (cloud m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C21C0_3DR>) ostream)
  "Serializes a message object of type '<C21C0_3DR>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'cloud) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C21C0_3DR>) istream)
  "Deserializes a message object of type '<C21C0_3DR>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'cloud) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C21C0_3DR>)))
  "Returns string type for a message object of type '<C21C0_3DR>"
  "C21_VisionAndLidar/C21C0_3DR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C21C0_3DR)))
  "Returns string type for a message object of type 'C21C0_3DR"
  "C21_VisionAndLidar/C21C0_3DR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C21C0_3DR>)))
  "Returns md5sum for a message object of type '<C21C0_3DR>"
  "96cec5374164b3b3d1d7ef5d7628a7ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C21C0_3DR)))
  "Returns md5sum for a message object of type 'C21C0_3DR"
  "96cec5374164b3b3d1d7ef5d7628a7ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C21C0_3DR>)))
  "Returns full string definition for message of type '<C21C0_3DR>"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C21C0_3DR)))
  "Returns full string definition for message of type 'C21C0_3DR"
  (cl:format cl:nil "sensor_msgs/PointCloud2 cloud~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C21C0_3DR>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'cloud))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C21C0_3DR>))
  "Converts a ROS message object to a list"
  (cl:list 'C21C0_3DR
    (cl:cons ':cloud (cloud msg))
))
