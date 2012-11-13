; Auto-generated. Do not edit!


(cl:in-package C31_PathPlanner-msg)


;//! \htmlinclude ppRobotDimension.msg.html

(cl:defclass <ppRobotDimension> (roslisp-msg-protocol:ros-message)
  ((size
    :reader size
    :initarg :size
    :type cl:float
    :initform 0.0))
)

(cl:defclass ppRobotDimension (<ppRobotDimension>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ppRobotDimension>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ppRobotDimension)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-msg:<ppRobotDimension> is deprecated: use C31_PathPlanner-msg:ppRobotDimension instead.")))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <ppRobotDimension>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:size-val is deprecated.  Use C31_PathPlanner-msg:size instead.")
  (size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ppRobotDimension>) ostream)
  "Serializes a message object of type '<ppRobotDimension>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'size))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ppRobotDimension>) istream)
  "Deserializes a message object of type '<ppRobotDimension>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'size) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ppRobotDimension>)))
  "Returns string type for a message object of type '<ppRobotDimension>"
  "C31_PathPlanner/ppRobotDimension")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ppRobotDimension)))
  "Returns string type for a message object of type 'ppRobotDimension"
  "C31_PathPlanner/ppRobotDimension")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ppRobotDimension>)))
  "Returns md5sum for a message object of type '<ppRobotDimension>"
  "3aa6cfa06a5f47b42010d9cec2c18e56")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ppRobotDimension)))
  "Returns md5sum for a message object of type 'ppRobotDimension"
  "3aa6cfa06a5f47b42010d9cec2c18e56")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ppRobotDimension>)))
  "Returns full string definition for message of type '<ppRobotDimension>"
  (cl:format cl:nil "float64 size~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ppRobotDimension)))
  "Returns full string definition for message of type 'ppRobotDimension"
  (cl:format cl:nil "float64 size~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ppRobotDimension>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ppRobotDimension>))
  "Converts a ROS message object to a list"
  (cl:list 'ppRobotDimension
    (cl:cons ':size (size msg))
))
