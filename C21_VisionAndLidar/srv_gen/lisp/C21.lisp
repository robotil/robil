; Auto-generated. Do not edit!


(cl:in-package C21_VisionAndLidar-srv)


;//! \htmlinclude C21-request.msg.html

(cl:defclass <C21-request> (roslisp-msg-protocol:ros-message)
  ((azimuth_msg
    :reader azimuth_msg
    :initarg :azimuth_msg
    :type C21_VisionAndLidar-msg:C0C21_AZI
    :initform (cl:make-instance 'C21_VisionAndLidar-msg:C0C21_AZI))
   (camera_sample_rate_msg
    :reader camera_sample_rate_msg
    :initarg :camera_sample_rate_msg
    :type C21_VisionAndLidar-msg:C0C21_CAM
    :initform (cl:make-instance 'C21_VisionAndLidar-msg:C0C21_CAM))
   (laser_sample_rate_msg
    :reader laser_sample_rate_msg
    :initarg :laser_sample_rate_msg
    :type C21_VisionAndLidar-msg:C0C21_LAZ
    :initform (cl:make-instance 'C21_VisionAndLidar-msg:C0C21_LAZ))
   (output_image_size_msg
    :reader output_image_size_msg
    :initarg :output_image_size_msg
    :type C21_VisionAndLidar-msg:C0C21_SIZ
    :initform (cl:make-instance 'C21_VisionAndLidar-msg:C0C21_SIZ))
   (required_resolution_msg
    :reader required_resolution_msg
    :initarg :required_resolution_msg
    :type C21_VisionAndLidar-msg:C0C21_RES
    :initform (cl:make-instance 'C21_VisionAndLidar-msg:C0C21_RES)))
)

(cl:defclass C21-request (<C21-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C21-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C21-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C21_VisionAndLidar-srv:<C21-request> is deprecated: use C21_VisionAndLidar-srv:C21-request instead.")))

(cl:ensure-generic-function 'azimuth_msg-val :lambda-list '(m))
(cl:defmethod azimuth_msg-val ((m <C21-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-srv:azimuth_msg-val is deprecated.  Use C21_VisionAndLidar-srv:azimuth_msg instead.")
  (azimuth_msg m))

(cl:ensure-generic-function 'camera_sample_rate_msg-val :lambda-list '(m))
(cl:defmethod camera_sample_rate_msg-val ((m <C21-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-srv:camera_sample_rate_msg-val is deprecated.  Use C21_VisionAndLidar-srv:camera_sample_rate_msg instead.")
  (camera_sample_rate_msg m))

(cl:ensure-generic-function 'laser_sample_rate_msg-val :lambda-list '(m))
(cl:defmethod laser_sample_rate_msg-val ((m <C21-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-srv:laser_sample_rate_msg-val is deprecated.  Use C21_VisionAndLidar-srv:laser_sample_rate_msg instead.")
  (laser_sample_rate_msg m))

(cl:ensure-generic-function 'output_image_size_msg-val :lambda-list '(m))
(cl:defmethod output_image_size_msg-val ((m <C21-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-srv:output_image_size_msg-val is deprecated.  Use C21_VisionAndLidar-srv:output_image_size_msg instead.")
  (output_image_size_msg m))

(cl:ensure-generic-function 'required_resolution_msg-val :lambda-list '(m))
(cl:defmethod required_resolution_msg-val ((m <C21-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-srv:required_resolution_msg-val is deprecated.  Use C21_VisionAndLidar-srv:required_resolution_msg instead.")
  (required_resolution_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C21-request>) ostream)
  "Serializes a message object of type '<C21-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'azimuth_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_sample_rate_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'laser_sample_rate_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'output_image_size_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'required_resolution_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C21-request>) istream)
  "Deserializes a message object of type '<C21-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'azimuth_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_sample_rate_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'laser_sample_rate_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'output_image_size_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'required_resolution_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C21-request>)))
  "Returns string type for a service object of type '<C21-request>"
  "C21_VisionAndLidar/C21Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C21-request)))
  "Returns string type for a service object of type 'C21-request"
  "C21_VisionAndLidar/C21Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C21-request>)))
  "Returns md5sum for a message object of type '<C21-request>"
  "4e473a5daadf18cff103ca3edae75827")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C21-request)))
  "Returns md5sum for a message object of type 'C21-request"
  "4e473a5daadf18cff103ca3edae75827")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C21-request>)))
  "Returns full string definition for message of type '<C21-request>"
  (cl:format cl:nil "C21_VisionAndLidar/C0C21_AZI azimuth_msg~%C21_VisionAndLidar/C0C21_CAM camera_sample_rate_msg~%C21_VisionAndLidar/C0C21_LAZ laser_sample_rate_msg~%C21_VisionAndLidar/C0C21_SIZ output_image_size_msg~%C21_VisionAndLidar/C0C21_RES required_resolution_msg~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_AZI~%float32 azimuth~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_LAZ~%int32 sampleRatePerSec~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_SIZ~%int32 image_width~%int32 image_height~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_RES~%int32 resulution_width~%int32 resulution_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C21-request)))
  "Returns full string definition for message of type 'C21-request"
  (cl:format cl:nil "C21_VisionAndLidar/C0C21_AZI azimuth_msg~%C21_VisionAndLidar/C0C21_CAM camera_sample_rate_msg~%C21_VisionAndLidar/C0C21_LAZ laser_sample_rate_msg~%C21_VisionAndLidar/C0C21_SIZ output_image_size_msg~%C21_VisionAndLidar/C0C21_RES required_resolution_msg~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_AZI~%float32 azimuth~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_LAZ~%int32 sampleRatePerSec~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_SIZ~%int32 image_width~%int32 image_height~%~%================================================================================~%MSG: C21_VisionAndLidar/C0C21_RES~%int32 resulution_width~%int32 resulution_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C21-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'azimuth_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_sample_rate_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'laser_sample_rate_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'output_image_size_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'required_resolution_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C21-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C21-request
    (cl:cons ':azimuth_msg (azimuth_msg msg))
    (cl:cons ':camera_sample_rate_msg (camera_sample_rate_msg msg))
    (cl:cons ':laser_sample_rate_msg (laser_sample_rate_msg msg))
    (cl:cons ':output_image_size_msg (output_image_size_msg msg))
    (cl:cons ':required_resolution_msg (required_resolution_msg msg))
))
;//! \htmlinclude C21-response.msg.html

(cl:defclass <C21-response> (roslisp-msg-protocol:ros-message)
  ((scene_full_resolution_msg
    :reader scene_full_resolution_msg
    :initarg :scene_full_resolution_msg
    :type C21_VisionAndLidar-msg:C21C0_3DF
    :initform (cl:make-instance 'C21_VisionAndLidar-msg:C21C0_3DF))
   (scene_reduced_resolution_msg
    :reader scene_reduced_resolution_msg
    :initarg :scene_reduced_resolution_msg
    :type C21_VisionAndLidar-msg:C21C0_3DR
    :initform (cl:make-instance 'C21_VisionAndLidar-msg:C21C0_3DR)))
)

(cl:defclass C21-response (<C21-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C21-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C21-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C21_VisionAndLidar-srv:<C21-response> is deprecated: use C21_VisionAndLidar-srv:C21-response instead.")))

(cl:ensure-generic-function 'scene_full_resolution_msg-val :lambda-list '(m))
(cl:defmethod scene_full_resolution_msg-val ((m <C21-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-srv:scene_full_resolution_msg-val is deprecated.  Use C21_VisionAndLidar-srv:scene_full_resolution_msg instead.")
  (scene_full_resolution_msg m))

(cl:ensure-generic-function 'scene_reduced_resolution_msg-val :lambda-list '(m))
(cl:defmethod scene_reduced_resolution_msg-val ((m <C21-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-srv:scene_reduced_resolution_msg-val is deprecated.  Use C21_VisionAndLidar-srv:scene_reduced_resolution_msg instead.")
  (scene_reduced_resolution_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C21-response>) ostream)
  "Serializes a message object of type '<C21-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'scene_full_resolution_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'scene_reduced_resolution_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C21-response>) istream)
  "Deserializes a message object of type '<C21-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'scene_full_resolution_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'scene_reduced_resolution_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C21-response>)))
  "Returns string type for a service object of type '<C21-response>"
  "C21_VisionAndLidar/C21Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C21-response)))
  "Returns string type for a service object of type 'C21-response"
  "C21_VisionAndLidar/C21Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C21-response>)))
  "Returns md5sum for a message object of type '<C21-response>"
  "4e473a5daadf18cff103ca3edae75827")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C21-response)))
  "Returns md5sum for a message object of type 'C21-response"
  "4e473a5daadf18cff103ca3edae75827")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C21-response>)))
  "Returns full string definition for message of type '<C21-response>"
  (cl:format cl:nil "C21_VisionAndLidar/C21C0_3DF scene_full_resolution_msg~%C21_VisionAndLidar/C21C0_3DR scene_reduced_resolution_msg~%~%~%================================================================================~%MSG: C21_VisionAndLidar/C21C0_3DF~%sensor_msgs/PointCloud2 cloud~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: C21_VisionAndLidar/C21C0_3DR~%sensor_msgs/PointCloud2 cloud~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C21-response)))
  "Returns full string definition for message of type 'C21-response"
  (cl:format cl:nil "C21_VisionAndLidar/C21C0_3DF scene_full_resolution_msg~%C21_VisionAndLidar/C21C0_3DR scene_reduced_resolution_msg~%~%~%================================================================================~%MSG: C21_VisionAndLidar/C21C0_3DF~%sensor_msgs/PointCloud2 cloud~%~%================================================================================~%MSG: sensor_msgs/PointCloud2~%# This message holds a collection of N-dimensional points, which may~%# contain additional information such as normals, intensity, etc. The~%# point data is stored as a binary blob, its layout described by the~%# contents of the \"fields\" array.~%~%# The point cloud data may be organized 2d (image-like) or 1d~%# (unordered). Point clouds organized as 2d images may be produced by~%# camera depth sensors such as stereo or time-of-flight.~%~%# Time of sensor data acquisition, and the coordinate frame ID (for 3d~%# points).~%Header header~%~%# 2D structure of the point cloud. If the cloud is unordered, height is~%# 1 and width is the length of the point cloud.~%uint32 height~%uint32 width~%~%# Describes the channels and their layout in the binary data blob.~%PointField[] fields~%~%bool    is_bigendian # Is this data bigendian?~%uint32  point_step   # Length of a point in bytes~%uint32  row_step     # Length of a row in bytes~%uint8[] data         # Actual point data, size is (row_step*height)~%~%bool is_dense        # True if there are no invalid points~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: sensor_msgs/PointField~%# This message holds the description of one point entry in the~%# PointCloud2 message format.~%uint8 INT8    = 1~%uint8 UINT8   = 2~%uint8 INT16   = 3~%uint8 UINT16  = 4~%uint8 INT32   = 5~%uint8 UINT32  = 6~%uint8 FLOAT32 = 7~%uint8 FLOAT64 = 8~%~%string name      # Name of field~%uint32 offset    # Offset from start of point struct~%uint8  datatype  # Datatype enumeration, see above~%uint32 count     # How many elements in the field~%~%================================================================================~%MSG: C21_VisionAndLidar/C21C0_3DR~%sensor_msgs/PointCloud2 cloud~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C21-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'scene_full_resolution_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'scene_reduced_resolution_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C21-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C21-response
    (cl:cons ':scene_full_resolution_msg (scene_full_resolution_msg msg))
    (cl:cons ':scene_reduced_resolution_msg (scene_reduced_resolution_msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C21)))
  'C21-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C21)))
  'C21-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C21)))
  "Returns string type for a service object of type '<C21>"
  "C21_VisionAndLidar/C21")