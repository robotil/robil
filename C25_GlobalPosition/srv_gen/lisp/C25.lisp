; Auto-generated. Do not edit!


(cl:in-package C25_GlobalPosition-srv)


;//! \htmlinclude C25-request.msg.html

(cl:defclass <C25-request> (roslisp-msg-protocol:ros-message)
  ((azimuth
    :reader azimuth
    :initarg :azimuth
    :type C25_GlobalPosition-msg:C0C25_AZI
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C0C25_AZI))
   (camera_sample_rate
    :reader camera_sample_rate
    :initarg :camera_sample_rate
    :type C25_GlobalPosition-msg:C0C25_CAM
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C0C25_CAM))
   (laser_sample_rate
    :reader laser_sample_rate
    :initarg :laser_sample_rate
    :type C25_GlobalPosition-msg:C0C25_LAZ
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C0C25_LAZ)))
)

(cl:defclass C25-request (<C25-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C25-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C25-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C25_GlobalPosition-srv:<C25-request> is deprecated: use C25_GlobalPosition-srv:C25-request instead.")))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <C25-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:azimuth-val is deprecated.  Use C25_GlobalPosition-srv:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'camera_sample_rate-val :lambda-list '(m))
(cl:defmethod camera_sample_rate-val ((m <C25-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:camera_sample_rate-val is deprecated.  Use C25_GlobalPosition-srv:camera_sample_rate instead.")
  (camera_sample_rate m))

(cl:ensure-generic-function 'laser_sample_rate-val :lambda-list '(m))
(cl:defmethod laser_sample_rate-val ((m <C25-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:laser_sample_rate-val is deprecated.  Use C25_GlobalPosition-srv:laser_sample_rate instead.")
  (laser_sample_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C25-request>) ostream)
  "Serializes a message object of type '<C25-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'azimuth) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_sample_rate) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'laser_sample_rate) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C25-request>) istream)
  "Deserializes a message object of type '<C25-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'azimuth) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_sample_rate) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'laser_sample_rate) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C25-request>)))
  "Returns string type for a service object of type '<C25-request>"
  "C25_GlobalPosition/C25Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C25-request)))
  "Returns string type for a service object of type 'C25-request"
  "C25_GlobalPosition/C25Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C25-request>)))
  "Returns md5sum for a message object of type '<C25-request>"
  "053887982b6555df9b370f618535b17e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C25-request)))
  "Returns md5sum for a message object of type 'C25-request"
  "053887982b6555df9b370f618535b17e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C25-request>)))
  "Returns full string definition for message of type '<C25-request>"
  (cl:format cl:nil "C25_GlobalPosition/C0C25_AZI azimuth~%C25_GlobalPosition/C0C25_CAM camera_sample_rate~%C25_GlobalPosition/C0C25_LAZ laser_sample_rate~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_AZI~%float32 azimuth~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_LAZ~%int32 sampleRatePerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C25-request)))
  "Returns full string definition for message of type 'C25-request"
  (cl:format cl:nil "C25_GlobalPosition/C0C25_AZI azimuth~%C25_GlobalPosition/C0C25_CAM camera_sample_rate~%C25_GlobalPosition/C0C25_LAZ laser_sample_rate~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_AZI~%float32 azimuth~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_LAZ~%int32 sampleRatePerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C25-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'azimuth))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_sample_rate))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'laser_sample_rate))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C25-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C25-request
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':camera_sample_rate (camera_sample_rate msg))
    (cl:cons ':laser_sample_rate (laser_sample_rate msg))
))
;//! \htmlinclude C25-response.msg.html

(cl:defclass <C25-response> (roslisp-msg-protocol:ros-message)
  ((robotPosition
    :reader robotPosition
    :initarg :robotPosition
    :type C25_GlobalPosition-msg:C25C0_ROP
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C25C0_ROP))
   (qualityOfError
    :reader qualityOfError
    :initarg :qualityOfError
    :type C25_GlobalPosition-msg:C25C0_OPO
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C25C0_OPO)))
)

(cl:defclass C25-response (<C25-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C25-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C25-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C25_GlobalPosition-srv:<C25-response> is deprecated: use C25_GlobalPosition-srv:C25-response instead.")))

(cl:ensure-generic-function 'robotPosition-val :lambda-list '(m))
(cl:defmethod robotPosition-val ((m <C25-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:robotPosition-val is deprecated.  Use C25_GlobalPosition-srv:robotPosition instead.")
  (robotPosition m))

(cl:ensure-generic-function 'qualityOfError-val :lambda-list '(m))
(cl:defmethod qualityOfError-val ((m <C25-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:qualityOfError-val is deprecated.  Use C25_GlobalPosition-srv:qualityOfError instead.")
  (qualityOfError m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C25-response>) ostream)
  "Serializes a message object of type '<C25-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robotPosition) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'qualityOfError) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C25-response>) istream)
  "Deserializes a message object of type '<C25-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robotPosition) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'qualityOfError) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C25-response>)))
  "Returns string type for a service object of type '<C25-response>"
  "C25_GlobalPosition/C25Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C25-response)))
  "Returns string type for a service object of type 'C25-response"
  "C25_GlobalPosition/C25Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C25-response>)))
  "Returns md5sum for a message object of type '<C25-response>"
  "053887982b6555df9b370f618535b17e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C25-response)))
  "Returns md5sum for a message object of type 'C25-response"
  "053887982b6555df9b370f618535b17e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C25-response>)))
  "Returns full string definition for message of type '<C25-response>"
  (cl:format cl:nil "C25_GlobalPosition/C25C0_ROP robotPosition~%C25_GlobalPosition/C25C0_OPO qualityOfError~%~%~%================================================================================~%MSG: C25_GlobalPosition/C25C0_ROP~%sensor_msgs/Imu imu~%nav_msgs/Odometry pose~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: C25_GlobalPosition/C25C0_OPO~%float32 qualityOfPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C25-response)))
  "Returns full string definition for message of type 'C25-response"
  (cl:format cl:nil "C25_GlobalPosition/C25C0_ROP robotPosition~%C25_GlobalPosition/C25C0_OPO qualityOfError~%~%~%================================================================================~%MSG: C25_GlobalPosition/C25C0_ROP~%sensor_msgs/Imu imu~%nav_msgs/Odometry pose~%================================================================================~%MSG: sensor_msgs/Imu~%# This is a message to hold data from an IMU (Inertial Measurement Unit)~%#~%# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec~%#~%# If the covariance of the measurement is known, it should be filled in (if all you know is the variance of each measurement, e.g. from the datasheet, just put those along the diagonal)~%# A covariance matrix of all zeros will be interpreted as \"covariance unknown\", and to use the data a covariance will have to be assumed or gotten from some other source~%#~%# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an orientation estimate), please set element 0 of the associated covariance matrix to -1~%# If you are interpreting this message, please check for a value of -1 in the first element of each covariance matrix, and disregard the associated estimate.~%~%Header header~%~%geometry_msgs/Quaternion orientation~%float64[9] orientation_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 angular_velocity~%float64[9] angular_velocity_covariance # Row major about x, y, z axes~%~%geometry_msgs/Vector3 linear_acceleration~%float64[9] linear_acceleration_covariance # Row major x, y z ~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: nav_msgs/Odometry~%# This represents an estimate of a position and velocity in free space.  ~%# The pose in this message should be specified in the coordinate frame given by header.frame_id.~%# The twist in this message should be specified in the coordinate frame given by the child_frame_id~%Header header~%string child_frame_id~%geometry_msgs/PoseWithCovariance pose~%geometry_msgs/TwistWithCovariance twist~%~%================================================================================~%MSG: geometry_msgs/PoseWithCovariance~%# This represents a pose in free space with uncertainty.~%~%Pose pose~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of postion and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/TwistWithCovariance~%# This expresses velocity in free space with uncertianty.~%~%Twist twist~%~%# Row-major representation of the 6x6 covariance matrix~%# The orientation parameters use a fixed-axis representation.~%# In order, the parameters are:~%# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)~%float64[36] covariance~%~%================================================================================~%MSG: geometry_msgs/Twist~%# This expresses velocity in free space broken into it's linear and angular parts. ~%Vector3  linear~%Vector3  angular~%~%================================================================================~%MSG: C25_GlobalPosition/C25C0_OPO~%float32 qualityOfPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C25-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robotPosition))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'qualityOfError))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C25-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C25-response
    (cl:cons ':robotPosition (robotPosition msg))
    (cl:cons ':qualityOfError (qualityOfError msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C25)))
  'C25-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C25)))
  'C25-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C25)))
  "Returns string type for a service object of type '<C25>"
  "C25_GlobalPosition/C25")