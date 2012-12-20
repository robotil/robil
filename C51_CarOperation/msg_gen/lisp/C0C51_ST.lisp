; Auto-generated. Do not edit!


(cl:in-package C51_CarOperation-msg)


;//! \htmlinclude C0C51_ST.msg.html

(cl:defclass <C0C51_ST> (roslisp-msg-protocol:ros-message)
  ((car_start_stop
    :reader car_start_stop
    :initarg :car_start_stop
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C51_ST (<C0C51_ST>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C51_ST>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C51_ST)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C51_CarOperation-msg:<C0C51_ST> is deprecated: use C51_CarOperation-msg:C0C51_ST instead.")))

(cl:ensure-generic-function 'car_start_stop-val :lambda-list '(m))
(cl:defmethod car_start_stop-val ((m <C0C51_ST>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:car_start_stop-val is deprecated.  Use C51_CarOperation-msg:car_start_stop instead.")
  (car_start_stop m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C0C51_ST>)))
    "Constants for message type '<C0C51_ST>"
  '((:START_CAR . 1)
    (:STOP_CAR . 0))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C0C51_ST)))
    "Constants for message type 'C0C51_ST"
  '((:START_CAR . 1)
    (:STOP_CAR . 0))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C51_ST>) ostream)
  "Serializes a message object of type '<C0C51_ST>"
  (cl:let* ((signed (cl:slot-value msg 'car_start_stop)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C51_ST>) istream)
  "Deserializes a message object of type '<C0C51_ST>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'car_start_stop) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C51_ST>)))
  "Returns string type for a message object of type '<C0C51_ST>"
  "C51_CarOperation/C0C51_ST")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C51_ST)))
  "Returns string type for a message object of type 'C0C51_ST"
  "C51_CarOperation/C0C51_ST")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C51_ST>)))
  "Returns md5sum for a message object of type '<C0C51_ST>"
  "026fc3434718efcfbf63a9a40356a01e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C51_ST)))
  "Returns md5sum for a message object of type 'C0C51_ST"
  "026fc3434718efcfbf63a9a40356a01e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C51_ST>)))
  "Returns full string definition for message of type '<C0C51_ST>"
  (cl:format cl:nil "int32 car_start_stop~%int32 START_CAR=1~%int32 STOP_CAR=0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C51_ST)))
  "Returns full string definition for message of type 'C0C51_ST"
  (cl:format cl:nil "int32 car_start_stop~%int32 START_CAR=1~%int32 STOP_CAR=0~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C51_ST>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C51_ST>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C51_ST
    (cl:cons ':car_start_stop (car_start_stop msg))
))
