; Auto-generated. Do not edit!


(cl:in-package C51_CarOperation-msg)


;//! \htmlinclude C51C0_NOR.msg.html

(cl:defclass <C51C0_NOR> (roslisp-msg-protocol:ros-message)
  ((normal_abnormal_travel
    :reader normal_abnormal_travel
    :initarg :normal_abnormal_travel
    :type cl:integer
    :initform 0))
)

(cl:defclass C51C0_NOR (<C51C0_NOR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C51C0_NOR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C51C0_NOR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C51_CarOperation-msg:<C51C0_NOR> is deprecated: use C51_CarOperation-msg:C51C0_NOR instead.")))

(cl:ensure-generic-function 'normal_abnormal_travel-val :lambda-list '(m))
(cl:defmethod normal_abnormal_travel-val ((m <C51C0_NOR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:normal_abnormal_travel-val is deprecated.  Use C51_CarOperation-msg:normal_abnormal_travel instead.")
  (normal_abnormal_travel m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C51C0_NOR>)))
    "Constants for message type '<C51C0_NOR>"
  '((:NORMAL_TRAVEL . 0)
    (:ABNORMAL_TRAVEL . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C51C0_NOR)))
    "Constants for message type 'C51C0_NOR"
  '((:NORMAL_TRAVEL . 0)
    (:ABNORMAL_TRAVEL . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C51C0_NOR>) ostream)
  "Serializes a message object of type '<C51C0_NOR>"
  (cl:let* ((signed (cl:slot-value msg 'normal_abnormal_travel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C51C0_NOR>) istream)
  "Deserializes a message object of type '<C51C0_NOR>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'normal_abnormal_travel) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C51C0_NOR>)))
  "Returns string type for a message object of type '<C51C0_NOR>"
  "C51_CarOperation/C51C0_NOR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C51C0_NOR)))
  "Returns string type for a message object of type 'C51C0_NOR"
  "C51_CarOperation/C51C0_NOR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C51C0_NOR>)))
  "Returns md5sum for a message object of type '<C51C0_NOR>"
  "239a790af8bbde68db44417642b6eec8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C51C0_NOR)))
  "Returns md5sum for a message object of type 'C51C0_NOR"
  "239a790af8bbde68db44417642b6eec8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C51C0_NOR>)))
  "Returns full string definition for message of type '<C51C0_NOR>"
  (cl:format cl:nil "int32 normal_abnormal_travel~%int32 NORMAL_TRAVEL=0~%int32 ABNORMAL_TRAVEL=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C51C0_NOR)))
  "Returns full string definition for message of type 'C51C0_NOR"
  (cl:format cl:nil "int32 normal_abnormal_travel~%int32 NORMAL_TRAVEL=0~%int32 ABNORMAL_TRAVEL=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C51C0_NOR>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C51C0_NOR>))
  "Converts a ROS message object to a list"
  (cl:list 'C51C0_NOR
    (cl:cons ':normal_abnormal_travel (normal_abnormal_travel msg))
))
