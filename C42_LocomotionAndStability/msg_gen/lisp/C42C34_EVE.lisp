; Auto-generated. Do not edit!


(cl:in-package C42_LocomotionAndStability-msg)


;//! \htmlinclude C42C34_EVE.msg.html

(cl:defclass <C42C34_EVE> (roslisp-msg-protocol:ros-message)
  ((Events
    :reader Events
    :initarg :Events
    :type cl:integer
    :initform 0))
)

(cl:defclass C42C34_EVE (<C42C34_EVE>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C42C34_EVE>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C42C34_EVE)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C42_LocomotionAndStability-msg:<C42C34_EVE> is deprecated: use C42_LocomotionAndStability-msg:C42C34_EVE instead.")))

(cl:ensure-generic-function 'Events-val :lambda-list '(m))
(cl:defmethod Events-val ((m <C42C34_EVE>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C42_LocomotionAndStability-msg:Events-val is deprecated.  Use C42_LocomotionAndStability-msg:Events instead.")
  (Events m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C42C34_EVE>)))
    "Constants for message type '<C42C34_EVE>"
  '((:FINISHED_TASK . 1)
    (:ROBOT_FELL . 2)
    (:PERFORMANCE_DEGRADATION . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C42C34_EVE)))
    "Constants for message type 'C42C34_EVE"
  '((:FINISHED_TASK . 1)
    (:ROBOT_FELL . 2)
    (:PERFORMANCE_DEGRADATION . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C42C34_EVE>) ostream)
  "Serializes a message object of type '<C42C34_EVE>"
  (cl:let* ((signed (cl:slot-value msg 'Events)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C42C34_EVE>) istream)
  "Deserializes a message object of type '<C42C34_EVE>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Events) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C42C34_EVE>)))
  "Returns string type for a message object of type '<C42C34_EVE>"
  "C42_LocomotionAndStability/C42C34_EVE")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C42C34_EVE)))
  "Returns string type for a message object of type 'C42C34_EVE"
  "C42_LocomotionAndStability/C42C34_EVE")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C42C34_EVE>)))
  "Returns md5sum for a message object of type '<C42C34_EVE>"
  "902aea6137c447505d551e9d8fec0e15")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C42C34_EVE)))
  "Returns md5sum for a message object of type 'C42C34_EVE"
  "902aea6137c447505d551e9d8fec0e15")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C42C34_EVE>)))
  "Returns full string definition for message of type '<C42C34_EVE>"
  (cl:format cl:nil "int32 Events~%int32 Finished_task=1~%int32 Robot_fell=2~%int32 Performance_degradation=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C42C34_EVE)))
  "Returns full string definition for message of type 'C42C34_EVE"
  (cl:format cl:nil "int32 Events~%int32 Finished_task=1~%int32 Robot_fell=2~%int32 Performance_degradation=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C42C34_EVE>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C42C34_EVE>))
  "Converts a ROS message object to a list"
  (cl:list 'C42C34_EVE
    (cl:cons ':Events (Events msg))
))
