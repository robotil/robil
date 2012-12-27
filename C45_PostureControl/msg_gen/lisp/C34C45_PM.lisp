; Auto-generated. Do not edit!


(cl:in-package C45_PostureControl-msg)


;//! \htmlinclude C34C45_PM.msg.html

(cl:defclass <C34C45_PM> (roslisp-msg-protocol:ros-message)
  ((posture_mode
    :reader posture_mode
    :initarg :posture_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass C34C45_PM (<C34C45_PM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C34C45_PM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C34C45_PM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C45_PostureControl-msg:<C34C45_PM> is deprecated: use C45_PostureControl-msg:C34C45_PM instead.")))

(cl:ensure-generic-function 'posture_mode-val :lambda-list '(m))
(cl:defmethod posture_mode-val ((m <C34C45_PM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C45_PostureControl-msg:posture_mode-val is deprecated.  Use C45_PostureControl-msg:posture_mode instead.")
  (posture_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C34C45_PM>) ostream)
  "Serializes a message object of type '<C34C45_PM>"
  (cl:let* ((signed (cl:slot-value msg 'posture_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C34C45_PM>) istream)
  "Deserializes a message object of type '<C34C45_PM>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'posture_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C34C45_PM>)))
  "Returns string type for a message object of type '<C34C45_PM>"
  "C45_PostureControl/C34C45_PM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C34C45_PM)))
  "Returns string type for a message object of type 'C34C45_PM"
  "C45_PostureControl/C34C45_PM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C34C45_PM>)))
  "Returns md5sum for a message object of type '<C34C45_PM>"
  "a2c997418f3151fbaafd1f219327a62d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C34C45_PM)))
  "Returns md5sum for a message object of type 'C34C45_PM"
  "a2c997418f3151fbaafd1f219327a62d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C34C45_PM>)))
  "Returns full string definition for message of type '<C34C45_PM>"
  (cl:format cl:nil "int32 posture_mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C34C45_PM)))
  "Returns full string definition for message of type 'C34C45_PM"
  (cl:format cl:nil "int32 posture_mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C34C45_PM>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C34C45_PM>))
  "Converts a ROS message object to a list"
  (cl:list 'C34C45_PM
    (cl:cons ':posture_mode (posture_mode msg))
))
