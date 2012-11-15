; Auto-generated. Do not edit!


(cl:in-package C42_LocomotionAndStability-msg)


;//! \htmlinclude C42C34_CS.msg.html

(cl:defclass <C42C34_CS> (roslisp-msg-protocol:ros-message)
  ((Current_Work_mode
    :reader Current_Work_mode
    :initarg :Current_Work_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass C42C34_CS (<C42C34_CS>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C42C34_CS>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C42C34_CS)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C42_LocomotionAndStability-msg:<C42C34_CS> is deprecated: use C42_LocomotionAndStability-msg:C42C34_CS instead.")))

(cl:ensure-generic-function 'Current_Work_mode-val :lambda-list '(m))
(cl:defmethod Current_Work_mode-val ((m <C42C34_CS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C42_LocomotionAndStability-msg:Current_Work_mode-val is deprecated.  Use C42_LocomotionAndStability-msg:Current_Work_mode instead.")
  (Current_Work_mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C42C34_CS>) ostream)
  "Serializes a message object of type '<C42C34_CS>"
  (cl:let* ((signed (cl:slot-value msg 'Current_Work_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C42C34_CS>) istream)
  "Deserializes a message object of type '<C42C34_CS>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Current_Work_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C42C34_CS>)))
  "Returns string type for a message object of type '<C42C34_CS>"
  "C42_LocomotionAndStability/C42C34_CS")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C42C34_CS)))
  "Returns string type for a message object of type 'C42C34_CS"
  "C42_LocomotionAndStability/C42C34_CS")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C42C34_CS>)))
  "Returns md5sum for a message object of type '<C42C34_CS>"
  "ec759d9d3247a59c422ebbc9f9301793")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C42C34_CS)))
  "Returns md5sum for a message object of type 'C42C34_CS"
  "ec759d9d3247a59c422ebbc9f9301793")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C42C34_CS>)))
  "Returns full string definition for message of type '<C42C34_CS>"
  (cl:format cl:nil "int32 Current_Work_mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C42C34_CS)))
  "Returns full string definition for message of type 'C42C34_CS"
  (cl:format cl:nil "int32 Current_Work_mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C42C34_CS>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C42C34_CS>))
  "Converts a ROS message object to a list"
  (cl:list 'C42C34_CS
    (cl:cons ':Current_Work_mode (Current_Work_mode msg))
))
