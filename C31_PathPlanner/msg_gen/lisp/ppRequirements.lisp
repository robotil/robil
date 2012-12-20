; Auto-generated. Do not edit!


(cl:in-package C31_PathPlanner-msg)


;//! \htmlinclude ppRequirements.msg.html

(cl:defclass <ppRequirements> (roslisp-msg-protocol:ros-message)
  ((wpd
    :reader wpd
    :initarg :wpd
    :type cl:float
    :initform 0.0))
)

(cl:defclass ppRequirements (<ppRequirements>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ppRequirements>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ppRequirements)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-msg:<ppRequirements> is deprecated: use C31_PathPlanner-msg:ppRequirements instead.")))

(cl:ensure-generic-function 'wpd-val :lambda-list '(m))
(cl:defmethod wpd-val ((m <ppRequirements>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:wpd-val is deprecated.  Use C31_PathPlanner-msg:wpd instead.")
  (wpd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ppRequirements>) ostream)
  "Serializes a message object of type '<ppRequirements>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'wpd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ppRequirements>) istream)
  "Deserializes a message object of type '<ppRequirements>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'wpd) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ppRequirements>)))
  "Returns string type for a message object of type '<ppRequirements>"
  "C31_PathPlanner/ppRequirements")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ppRequirements)))
  "Returns string type for a message object of type 'ppRequirements"
  "C31_PathPlanner/ppRequirements")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ppRequirements>)))
  "Returns md5sum for a message object of type '<ppRequirements>"
  "f471b978d88d3a669ebf21591c05c7bc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ppRequirements)))
  "Returns md5sum for a message object of type 'ppRequirements"
  "f471b978d88d3a669ebf21591c05c7bc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ppRequirements>)))
  "Returns full string definition for message of type '<ppRequirements>"
  (cl:format cl:nil "float64 wpd~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ppRequirements)))
  "Returns full string definition for message of type 'ppRequirements"
  (cl:format cl:nil "float64 wpd~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ppRequirements>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ppRequirements>))
  "Converts a ROS message object to a list"
  (cl:list 'ppRequirements
    (cl:cons ':wpd (wpd msg))
))
