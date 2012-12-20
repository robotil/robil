; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude coord.msg.html

(cl:defclass <coord> (roslisp-msg-protocol:ros-message)
  ((X
    :reader X
    :initarg :X
    :type cl:float
    :initform 0.0)
   (Y
    :reader Y
    :initarg :Y
    :type cl:float
    :initform 0.0)
   (Z
    :reader Z
    :initarg :Z
    :type cl:float
    :initform 0.0))
)

(cl:defclass coord (<coord>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <coord>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'coord)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<coord> is deprecated: use C11_Agent-msg:coord instead.")))

(cl:ensure-generic-function 'X-val :lambda-list '(m))
(cl:defmethod X-val ((m <coord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:X-val is deprecated.  Use C11_Agent-msg:X instead.")
  (X m))

(cl:ensure-generic-function 'Y-val :lambda-list '(m))
(cl:defmethod Y-val ((m <coord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:Y-val is deprecated.  Use C11_Agent-msg:Y instead.")
  (Y m))

(cl:ensure-generic-function 'Z-val :lambda-list '(m))
(cl:defmethod Z-val ((m <coord>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:Z-val is deprecated.  Use C11_Agent-msg:Z instead.")
  (Z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <coord>) ostream)
  "Serializes a message object of type '<coord>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'X))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <coord>) istream)
  "Deserializes a message object of type '<coord>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Z) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<coord>)))
  "Returns string type for a message object of type '<coord>"
  "C11_Agent/coord")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'coord)))
  "Returns string type for a message object of type 'coord"
  "C11_Agent/coord")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<coord>)))
  "Returns md5sum for a message object of type '<coord>"
  "8219583d7802cc50be3e9ab911877ba5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'coord)))
  "Returns md5sum for a message object of type 'coord"
  "8219583d7802cc50be3e9ab911877ba5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<coord>)))
  "Returns full string definition for message of type '<coord>"
  (cl:format cl:nil "float64 X~%float64 Y~%float64 Z~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'coord)))
  "Returns full string definition for message of type 'coord"
  (cl:format cl:nil "float64 X~%float64 Y~%float64 Z~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <coord>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <coord>))
  "Converts a ROS message object to a list"
  (cl:list 'coord
    (cl:cons ':X (X msg))
    (cl:cons ':Y (Y msg))
    (cl:cons ':Z (Z msg))
))
