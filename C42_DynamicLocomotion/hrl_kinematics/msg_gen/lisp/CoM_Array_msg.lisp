; Auto-generated. Do not edit!


(cl:in-package hrl_kinematics-msg)


;//! \htmlinclude CoM_Array_msg.msg.html

(cl:defclass <CoM_Array_msg> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (x
    :reader x
    :initarg :x
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (y
    :reader y
    :initarg :y
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (z
    :reader z
    :initarg :z
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass CoM_Array_msg (<CoM_Array_msg>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CoM_Array_msg>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CoM_Array_msg)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hrl_kinematics-msg:<CoM_Array_msg> is deprecated: use hrl_kinematics-msg:CoM_Array_msg instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <CoM_Array_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrl_kinematics-msg:header-val is deprecated.  Use hrl_kinematics-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <CoM_Array_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrl_kinematics-msg:x-val is deprecated.  Use hrl_kinematics-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <CoM_Array_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrl_kinematics-msg:y-val is deprecated.  Use hrl_kinematics-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <CoM_Array_msg>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrl_kinematics-msg:z-val is deprecated.  Use hrl_kinematics-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CoM_Array_msg>) ostream)
  "Serializes a message object of type '<CoM_Array_msg>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'x))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'y))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'z))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CoM_Array_msg>) istream)
  "Deserializes a message object of type '<CoM_Array_msg>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:setf (cl:slot-value msg 'x) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'x)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'y) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'y)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'z) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'z)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CoM_Array_msg>)))
  "Returns string type for a message object of type '<CoM_Array_msg>"
  "hrl_kinematics/CoM_Array_msg")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CoM_Array_msg)))
  "Returns string type for a message object of type 'CoM_Array_msg"
  "hrl_kinematics/CoM_Array_msg")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CoM_Array_msg>)))
  "Returns md5sum for a message object of type '<CoM_Array_msg>"
  "458d6ca638eb99402ee2d03fb6bc79ba")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CoM_Array_msg)))
  "Returns md5sum for a message object of type 'CoM_Array_msg"
  "458d6ca638eb99402ee2d03fb6bc79ba")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CoM_Array_msg>)))
  "Returns full string definition for message of type '<CoM_Array_msg>"
  (cl:format cl:nil "# This contains the position of 3 points that indicate the position~%# of the Center of Mass in three different frames of coordinates:~%# The first (num. 0) gives the CoM position in the root frame (\"pelvis\").~%# The second (num. 1) gives the CoM position in the right foot frame (\"r_foot\"). ~%# The third (num. 2) gives the CoM position in the left foot frame (\"l_foot\").~%~%# geometry_msgs/Point root~%# geometry_msgs/Point Rfoot~%# geometry_msgs/Point Lfoot~%~%Header header~%~%float64[3] x ~%float64[3] y~%float64[3] z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CoM_Array_msg)))
  "Returns full string definition for message of type 'CoM_Array_msg"
  (cl:format cl:nil "# This contains the position of 3 points that indicate the position~%# of the Center of Mass in three different frames of coordinates:~%# The first (num. 0) gives the CoM position in the root frame (\"pelvis\").~%# The second (num. 1) gives the CoM position in the right foot frame (\"r_foot\"). ~%# The third (num. 2) gives the CoM position in the left foot frame (\"l_foot\").~%~%# geometry_msgs/Point root~%# geometry_msgs/Point Rfoot~%# geometry_msgs/Point Lfoot~%~%Header header~%~%float64[3] x ~%float64[3] y~%float64[3] z~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CoM_Array_msg>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'x) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'y) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'z) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CoM_Array_msg>))
  "Converts a ROS message object to a list"
  (cl:list 'CoM_Array_msg
    (cl:cons ':header (header msg))
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
