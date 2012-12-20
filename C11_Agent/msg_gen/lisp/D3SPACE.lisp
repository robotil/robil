; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude D3SPACE.msg.html

(cl:defclass <D3SPACE> (roslisp-msg-protocol:ros-message)
  ((ROLL
    :reader ROLL
    :initarg :ROLL
    :type cl:float
    :initform 0.0)
   (PITCH
    :reader PITCH
    :initarg :PITCH
    :type cl:float
    :initform 0.0)
   (YAW
    :reader YAW
    :initarg :YAW
    :type cl:float
    :initform 0.0))
)

(cl:defclass D3SPACE (<D3SPACE>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <D3SPACE>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'D3SPACE)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<D3SPACE> is deprecated: use C11_Agent-msg:D3SPACE instead.")))

(cl:ensure-generic-function 'ROLL-val :lambda-list '(m))
(cl:defmethod ROLL-val ((m <D3SPACE>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ROLL-val is deprecated.  Use C11_Agent-msg:ROLL instead.")
  (ROLL m))

(cl:ensure-generic-function 'PITCH-val :lambda-list '(m))
(cl:defmethod PITCH-val ((m <D3SPACE>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:PITCH-val is deprecated.  Use C11_Agent-msg:PITCH instead.")
  (PITCH m))

(cl:ensure-generic-function 'YAW-val :lambda-list '(m))
(cl:defmethod YAW-val ((m <D3SPACE>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:YAW-val is deprecated.  Use C11_Agent-msg:YAW instead.")
  (YAW m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <D3SPACE>) ostream)
  "Serializes a message object of type '<D3SPACE>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'ROLL))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'PITCH))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'YAW))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <D3SPACE>) istream)
  "Deserializes a message object of type '<D3SPACE>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'ROLL) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PITCH) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'YAW) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<D3SPACE>)))
  "Returns string type for a message object of type '<D3SPACE>"
  "C11_Agent/D3SPACE")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'D3SPACE)))
  "Returns string type for a message object of type 'D3SPACE"
  "C11_Agent/D3SPACE")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<D3SPACE>)))
  "Returns md5sum for a message object of type '<D3SPACE>"
  "5e410119c3186b5dbf9b94f595628907")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'D3SPACE)))
  "Returns md5sum for a message object of type 'D3SPACE"
  "5e410119c3186b5dbf9b94f595628907")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<D3SPACE>)))
  "Returns full string definition for message of type '<D3SPACE>"
  (cl:format cl:nil "float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'D3SPACE)))
  "Returns full string definition for message of type 'D3SPACE"
  (cl:format cl:nil "float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <D3SPACE>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <D3SPACE>))
  "Converts a ROS message object to a list"
  (cl:list 'D3SPACE
    (cl:cons ':ROLL (ROLL msg))
    (cl:cons ':PITCH (PITCH msg))
    (cl:cons ':YAW (YAW msg))
))
