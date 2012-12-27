; Auto-generated. Do not edit!


(cl:in-package C45_PostureControl-msg)


;//! \htmlinclude C34C45_PSU.msg.html

(cl:defclass <C34C45_PSU> (roslisp-msg-protocol:ros-message)
  ((posture_state_update
    :reader posture_state_update
    :initarg :posture_state_update
    :type cl:integer
    :initform 0))
)

(cl:defclass C34C45_PSU (<C34C45_PSU>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C34C45_PSU>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C34C45_PSU)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C45_PostureControl-msg:<C34C45_PSU> is deprecated: use C45_PostureControl-msg:C34C45_PSU instead.")))

(cl:ensure-generic-function 'posture_state_update-val :lambda-list '(m))
(cl:defmethod posture_state_update-val ((m <C34C45_PSU>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C45_PostureControl-msg:posture_state_update-val is deprecated.  Use C45_PostureControl-msg:posture_state_update instead.")
  (posture_state_update m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C34C45_PSU>) ostream)
  "Serializes a message object of type '<C34C45_PSU>"
  (cl:let* ((signed (cl:slot-value msg 'posture_state_update)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C34C45_PSU>) istream)
  "Deserializes a message object of type '<C34C45_PSU>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'posture_state_update) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C34C45_PSU>)))
  "Returns string type for a message object of type '<C34C45_PSU>"
  "C45_PostureControl/C34C45_PSU")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C34C45_PSU)))
  "Returns string type for a message object of type 'C34C45_PSU"
  "C45_PostureControl/C34C45_PSU")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C34C45_PSU>)))
  "Returns md5sum for a message object of type '<C34C45_PSU>"
  "5d913d20b0c65c63caaf018ab5c6c2f0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C34C45_PSU)))
  "Returns md5sum for a message object of type 'C34C45_PSU"
  "5d913d20b0c65c63caaf018ab5c6c2f0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C34C45_PSU>)))
  "Returns full string definition for message of type '<C34C45_PSU>"
  (cl:format cl:nil "int32 posture_state_update~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C34C45_PSU)))
  "Returns full string definition for message of type 'C34C45_PSU"
  (cl:format cl:nil "int32 posture_state_update~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C34C45_PSU>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C34C45_PSU>))
  "Converts a ROS message object to a list"
  (cl:list 'C34C45_PSU
    (cl:cons ':posture_state_update (posture_state_update msg))
))
