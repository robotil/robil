; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C32C11_PATH.msg.html

(cl:defclass <C32C11_PATH> (roslisp-msg-protocol:ros-message)
  ((lat
    :reader lat
    :initarg :lat
    :type cl:float
    :initform 0.0)
   (lon
    :reader lon
    :initarg :lon
    :type cl:float
    :initform 0.0))
)

(cl:defclass C32C11_PATH (<C32C11_PATH>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C32C11_PATH>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C32C11_PATH)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C32C11_PATH> is deprecated: use C11_Agent-msg:C32C11_PATH instead.")))

(cl:ensure-generic-function 'lat-val :lambda-list '(m))
(cl:defmethod lat-val ((m <C32C11_PATH>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:lat-val is deprecated.  Use C11_Agent-msg:lat instead.")
  (lat m))

(cl:ensure-generic-function 'lon-val :lambda-list '(m))
(cl:defmethod lon-val ((m <C32C11_PATH>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:lon-val is deprecated.  Use C11_Agent-msg:lon instead.")
  (lon m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C32C11_PATH>) ostream)
  "Serializes a message object of type '<C32C11_PATH>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lat))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'lon))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C32C11_PATH>) istream)
  "Deserializes a message object of type '<C32C11_PATH>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lat) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'lon) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C32C11_PATH>)))
  "Returns string type for a message object of type '<C32C11_PATH>"
  "C11_Agent/C32C11_PATH")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C32C11_PATH)))
  "Returns string type for a message object of type 'C32C11_PATH"
  "C11_Agent/C32C11_PATH")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C32C11_PATH>)))
  "Returns md5sum for a message object of type '<C32C11_PATH>"
  "deb12644498d4b5511a84dbd9af1e283")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C32C11_PATH)))
  "Returns md5sum for a message object of type 'C32C11_PATH"
  "deb12644498d4b5511a84dbd9af1e283")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C32C11_PATH>)))
  "Returns full string definition for message of type '<C32C11_PATH>"
  (cl:format cl:nil "float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C32C11_PATH)))
  "Returns full string definition for message of type 'C32C11_PATH"
  (cl:format cl:nil "float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C32C11_PATH>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C32C11_PATH>))
  "Converts a ROS message object to a list"
  (cl:list 'C32C11_PATH
    (cl:cons ':lat (lat msg))
    (cl:cons ':lon (lon msg))
))
