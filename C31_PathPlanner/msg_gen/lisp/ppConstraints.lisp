; Auto-generated. Do not edit!


(cl:in-package C31_PathPlanner-msg)


;//! \htmlinclude ppConstraints.msg.html

(cl:defclass <ppConstraints> (roslisp-msg-protocol:ros-message)
  ((corridor
    :reader corridor
    :initarg :corridor
    :type C31_PathPlanner-msg:ppCorridor
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppCorridor))
   (robot
    :reader robot
    :initarg :robot
    :type C31_PathPlanner-msg:ppRobotDimension
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppRobotDimension))
   (attractors
    :reader attractors
    :initarg :attractors
    :type (cl:vector C31_PathPlanner-msg:ppCharge)
   :initform (cl:make-array 0 :element-type 'C31_PathPlanner-msg:ppCharge :initial-element (cl:make-instance 'C31_PathPlanner-msg:ppCharge)))
   (repulsors
    :reader repulsors
    :initarg :repulsors
    :type (cl:vector C31_PathPlanner-msg:ppCharge)
   :initform (cl:make-array 0 :element-type 'C31_PathPlanner-msg:ppCharge :initial-element (cl:make-instance 'C31_PathPlanner-msg:ppCharge))))
)

(cl:defclass ppConstraints (<ppConstraints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ppConstraints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ppConstraints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-msg:<ppConstraints> is deprecated: use C31_PathPlanner-msg:ppConstraints instead.")))

(cl:ensure-generic-function 'corridor-val :lambda-list '(m))
(cl:defmethod corridor-val ((m <ppConstraints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:corridor-val is deprecated.  Use C31_PathPlanner-msg:corridor instead.")
  (corridor m))

(cl:ensure-generic-function 'robot-val :lambda-list '(m))
(cl:defmethod robot-val ((m <ppConstraints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:robot-val is deprecated.  Use C31_PathPlanner-msg:robot instead.")
  (robot m))

(cl:ensure-generic-function 'attractors-val :lambda-list '(m))
(cl:defmethod attractors-val ((m <ppConstraints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:attractors-val is deprecated.  Use C31_PathPlanner-msg:attractors instead.")
  (attractors m))

(cl:ensure-generic-function 'repulsors-val :lambda-list '(m))
(cl:defmethod repulsors-val ((m <ppConstraints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:repulsors-val is deprecated.  Use C31_PathPlanner-msg:repulsors instead.")
  (repulsors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ppConstraints>) ostream)
  "Serializes a message object of type '<ppConstraints>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'corridor) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robot) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'attractors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'attractors))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'repulsors))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'repulsors))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ppConstraints>) istream)
  "Deserializes a message object of type '<ppConstraints>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'corridor) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robot) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'attractors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'attractors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'C31_PathPlanner-msg:ppCharge))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'repulsors) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'repulsors)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'C31_PathPlanner-msg:ppCharge))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ppConstraints>)))
  "Returns string type for a message object of type '<ppConstraints>"
  "C31_PathPlanner/ppConstraints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ppConstraints)))
  "Returns string type for a message object of type 'ppConstraints"
  "C31_PathPlanner/ppConstraints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ppConstraints>)))
  "Returns md5sum for a message object of type '<ppConstraints>"
  "7a1e2b18a0da9e12a6b2094e7a916c5c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ppConstraints)))
  "Returns md5sum for a message object of type 'ppConstraints"
  "7a1e2b18a0da9e12a6b2094e7a916c5c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ppConstraints>)))
  "Returns full string definition for message of type '<ppConstraints>"
  (cl:format cl:nil "C31_PathPlanner/ppCorridor corridor~%C31_PathPlanner/ppRobotDimension robot~%C31_PathPlanner/ppCharge[] attractors~%C31_PathPlanner/ppCharge[] repulsors~%~%================================================================================~%MSG: C31_PathPlanner/ppCorridor~%float64 width~%================================================================================~%MSG: C31_PathPlanner/ppRobotDimension~%float64 size~%================================================================================~%MSG: C31_PathPlanner/ppCharge~%C31_PathPlanner/ppLocation location~%float64 power~%~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ppConstraints)))
  "Returns full string definition for message of type 'ppConstraints"
  (cl:format cl:nil "C31_PathPlanner/ppCorridor corridor~%C31_PathPlanner/ppRobotDimension robot~%C31_PathPlanner/ppCharge[] attractors~%C31_PathPlanner/ppCharge[] repulsors~%~%================================================================================~%MSG: C31_PathPlanner/ppCorridor~%float64 width~%================================================================================~%MSG: C31_PathPlanner/ppRobotDimension~%float64 size~%================================================================================~%MSG: C31_PathPlanner/ppCharge~%C31_PathPlanner/ppLocation location~%float64 power~%~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ppConstraints>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'corridor))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robot))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'attractors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'repulsors) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ppConstraints>))
  "Converts a ROS message object to a list"
  (cl:list 'ppConstraints
    (cl:cons ':corridor (corridor msg))
    (cl:cons ':robot (robot msg))
    (cl:cons ':attractors (attractors msg))
    (cl:cons ':repulsors (repulsors msg))
))
