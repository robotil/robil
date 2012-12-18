; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C34C11_STT.msg.html

(cl:defclass <C34C11_STT> (roslisp-msg-protocol:ros-message)
  ((stt
    :reader stt
    :initarg :stt
    :type cl:fixnum
    :initform 0))
)

(cl:defclass C34C11_STT (<C34C11_STT>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C34C11_STT>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C34C11_STT)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C34C11_STT> is deprecated: use C11_Agent-msg:C34C11_STT instead.")))

(cl:ensure-generic-function 'stt-val :lambda-list '(m))
(cl:defmethod stt-val ((m <C34C11_STT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:stt-val is deprecated.  Use C11_Agent-msg:stt instead.")
  (stt m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C34C11_STT>) ostream)
  "Serializes a message object of type '<C34C11_STT>"
  (cl:let* ((signed (cl:slot-value msg 'stt)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C34C11_STT>) istream)
  "Deserializes a message object of type '<C34C11_STT>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'stt) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C34C11_STT>)))
  "Returns string type for a message object of type '<C34C11_STT>"
  "C11_Agent/C34C11_STT")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C34C11_STT)))
  "Returns string type for a message object of type 'C34C11_STT"
  "C11_Agent/C34C11_STT")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C34C11_STT>)))
  "Returns md5sum for a message object of type '<C34C11_STT>"
  "650018e0b258e00520bce2e8b307e19f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C34C11_STT)))
  "Returns md5sum for a message object of type 'C34C11_STT"
  "650018e0b258e00520bce2e8b307e19f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C34C11_STT>)))
  "Returns full string definition for message of type '<C34C11_STT>"
  (cl:format cl:nil "int8 stt~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C34C11_STT)))
  "Returns full string definition for message of type 'C34C11_STT"
  (cl:format cl:nil "int8 stt~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C34C11_STT>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C34C11_STT>))
  "Converts a ROS message object to a list"
  (cl:list 'C34C11_STT
    (cl:cons ':stt (stt msg))
))
