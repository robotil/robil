; Auto-generated. Do not edit!


(cl:in-package C25_GlobalPosition-msg)


;//! \htmlinclude C25C0_OPO.msg.html

(cl:defclass <C25C0_OPO> (roslisp-msg-protocol:ros-message)
  ((qualityOfPosition
    :reader qualityOfPosition
    :initarg :qualityOfPosition
    :type cl:float
    :initform 0.0))
)

(cl:defclass C25C0_OPO (<C25C0_OPO>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C25C0_OPO>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C25C0_OPO)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C25_GlobalPosition-msg:<C25C0_OPO> is deprecated: use C25_GlobalPosition-msg:C25C0_OPO instead.")))

(cl:ensure-generic-function 'qualityOfPosition-val :lambda-list '(m))
(cl:defmethod qualityOfPosition-val ((m <C25C0_OPO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-msg:qualityOfPosition-val is deprecated.  Use C25_GlobalPosition-msg:qualityOfPosition instead.")
  (qualityOfPosition m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C25C0_OPO>) ostream)
  "Serializes a message object of type '<C25C0_OPO>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'qualityOfPosition))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C25C0_OPO>) istream)
  "Deserializes a message object of type '<C25C0_OPO>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'qualityOfPosition) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C25C0_OPO>)))
  "Returns string type for a message object of type '<C25C0_OPO>"
  "C25_GlobalPosition/C25C0_OPO")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C25C0_OPO)))
  "Returns string type for a message object of type 'C25C0_OPO"
  "C25_GlobalPosition/C25C0_OPO")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C25C0_OPO>)))
  "Returns md5sum for a message object of type '<C25C0_OPO>"
  "0309c4c44808bf527ccce0bb94d66c08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C25C0_OPO)))
  "Returns md5sum for a message object of type 'C25C0_OPO"
  "0309c4c44808bf527ccce0bb94d66c08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C25C0_OPO>)))
  "Returns full string definition for message of type '<C25C0_OPO>"
  (cl:format cl:nil "float32 qualityOfPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C25C0_OPO)))
  "Returns full string definition for message of type 'C25C0_OPO"
  (cl:format cl:nil "float32 qualityOfPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C25C0_OPO>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C25C0_OPO>))
  "Converts a ROS message object to a list"
  (cl:list 'C25C0_OPO
    (cl:cons ':qualityOfPosition (qualityOfPosition msg))
))
