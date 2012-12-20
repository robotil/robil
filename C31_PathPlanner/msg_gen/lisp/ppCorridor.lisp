; Auto-generated. Do not edit!


(cl:in-package C31_PathPlanner-msg)


;//! \htmlinclude ppCorridor.msg.html

(cl:defclass <ppCorridor> (roslisp-msg-protocol:ros-message)
  ((width
    :reader width
    :initarg :width
    :type cl:float
    :initform 0.0))
)

(cl:defclass ppCorridor (<ppCorridor>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ppCorridor>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ppCorridor)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-msg:<ppCorridor> is deprecated: use C31_PathPlanner-msg:ppCorridor instead.")))

(cl:ensure-generic-function 'width-val :lambda-list '(m))
(cl:defmethod width-val ((m <ppCorridor>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:width-val is deprecated.  Use C31_PathPlanner-msg:width instead.")
  (width m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ppCorridor>) ostream)
  "Serializes a message object of type '<ppCorridor>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'width))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ppCorridor>) istream)
  "Deserializes a message object of type '<ppCorridor>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'width) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ppCorridor>)))
  "Returns string type for a message object of type '<ppCorridor>"
  "C31_PathPlanner/ppCorridor")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ppCorridor)))
  "Returns string type for a message object of type 'ppCorridor"
  "C31_PathPlanner/ppCorridor")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ppCorridor>)))
  "Returns md5sum for a message object of type '<ppCorridor>"
  "a334e8a8f988e31f739167a339fc51af")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ppCorridor)))
  "Returns md5sum for a message object of type 'ppCorridor"
  "a334e8a8f988e31f739167a339fc51af")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ppCorridor>)))
  "Returns full string definition for message of type '<ppCorridor>"
  (cl:format cl:nil "float64 width~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ppCorridor)))
  "Returns full string definition for message of type 'ppCorridor"
  (cl:format cl:nil "float64 width~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ppCorridor>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ppCorridor>))
  "Converts a ROS message object to a list"
  (cl:list 'ppCorridor
    (cl:cons ':width (width msg))
))
