; Auto-generated. Do not edit!


(cl:in-package C41_BodyControl-msg)


;//! \htmlinclude C0C41_WM.msg.html

(cl:defclass <C0C41_WM> (roslisp-msg-protocol:ros-message)
  ((Work_mode
    :reader Work_mode
    :initarg :Work_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C41_WM (<C0C41_WM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C41_WM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C41_WM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_BodyControl-msg:<C0C41_WM> is deprecated: use C41_BodyControl-msg:C0C41_WM instead.")))

(cl:ensure-generic-function 'Work_mode-val :lambda-list '(m))
(cl:defmethod Work_mode-val ((m <C0C41_WM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:Work_mode-val is deprecated.  Use C41_BodyControl-msg:Work_mode instead.")
  (Work_mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C0C41_WM>)))
    "Constants for message type '<C0C41_WM>"
  '((:TORQUE_CONTROL . 1)
    (:PVA_CONTROL . 2))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C0C41_WM)))
    "Constants for message type 'C0C41_WM"
  '((:TORQUE_CONTROL . 1)
    (:PVA_CONTROL . 2))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C41_WM>) ostream)
  "Serializes a message object of type '<C0C41_WM>"
  (cl:let* ((signed (cl:slot-value msg 'Work_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C41_WM>) istream)
  "Deserializes a message object of type '<C0C41_WM>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Work_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C41_WM>)))
  "Returns string type for a message object of type '<C0C41_WM>"
  "C41_BodyControl/C0C41_WM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C41_WM)))
  "Returns string type for a message object of type 'C0C41_WM"
  "C41_BodyControl/C0C41_WM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C41_WM>)))
  "Returns md5sum for a message object of type '<C0C41_WM>"
  "cd2e7d116c01bb5cd1733fe0ed106dc3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C41_WM)))
  "Returns md5sum for a message object of type 'C0C41_WM"
  "cd2e7d116c01bb5cd1733fe0ed106dc3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C41_WM>)))
  "Returns full string definition for message of type '<C0C41_WM>"
  (cl:format cl:nil "int32 Work_mode~%int32 Torque_Control=1~%int32 PVA_Control=2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C41_WM)))
  "Returns full string definition for message of type 'C0C41_WM"
  (cl:format cl:nil "int32 Work_mode~%int32 Torque_Control=1~%int32 PVA_Control=2~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C41_WM>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C41_WM>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C41_WM
    (cl:cons ':Work_mode (Work_mode msg))
))
