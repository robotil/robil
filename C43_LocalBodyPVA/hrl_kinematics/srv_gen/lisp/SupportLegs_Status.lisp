; Auto-generated. Do not edit!


(cl:in-package hrl_kinematics-srv)


;//! \htmlinclude SupportLegs_Status-request.msg.html

(cl:defclass <SupportLegs_Status-request> (roslisp-msg-protocol:ros-message)
  ((FootSupport_CMD
    :reader FootSupport_CMD
    :initarg :FootSupport_CMD
    :type cl:fixnum
    :initform 0))
)

(cl:defclass SupportLegs_Status-request (<SupportLegs_Status-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SupportLegs_Status-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SupportLegs_Status-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hrl_kinematics-srv:<SupportLegs_Status-request> is deprecated: use hrl_kinematics-srv:SupportLegs_Status-request instead.")))

(cl:ensure-generic-function 'FootSupport_CMD-val :lambda-list '(m))
(cl:defmethod FootSupport_CMD-val ((m <SupportLegs_Status-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrl_kinematics-srv:FootSupport_CMD-val is deprecated.  Use hrl_kinematics-srv:FootSupport_CMD instead.")
  (FootSupport_CMD m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<SupportLegs_Status-request>)))
    "Constants for message type '<SupportLegs_Status-request>"
  '((:SUPPORT_DOUBLE . 0)
    (:SUPPORT_SINGLE_RIGHT . 1)
    (:SUPPORT_SINGLE_LEFT . 2)
    (:GET_SUPPORT_STATUS . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'SupportLegs_Status-request)))
    "Constants for message type 'SupportLegs_Status-request"
  '((:SUPPORT_DOUBLE . 0)
    (:SUPPORT_SINGLE_RIGHT . 1)
    (:SUPPORT_SINGLE_LEFT . 2)
    (:GET_SUPPORT_STATUS . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SupportLegs_Status-request>) ostream)
  "Serializes a message object of type '<SupportLegs_Status-request>"
  (cl:let* ((signed (cl:slot-value msg 'FootSupport_CMD)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SupportLegs_Status-request>) istream)
  "Deserializes a message object of type '<SupportLegs_Status-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'FootSupport_CMD) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SupportLegs_Status-request>)))
  "Returns string type for a service object of type '<SupportLegs_Status-request>"
  "hrl_kinematics/SupportLegs_StatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SupportLegs_Status-request)))
  "Returns string type for a service object of type 'SupportLegs_Status-request"
  "hrl_kinematics/SupportLegs_StatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SupportLegs_Status-request>)))
  "Returns md5sum for a message object of type '<SupportLegs_Status-request>"
  "ec7194f86cdbf88898be0a72e8be6e9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SupportLegs_Status-request)))
  "Returns md5sum for a message object of type 'SupportLegs_Status-request"
  "ec7194f86cdbf88898be0a72e8be6e9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SupportLegs_Status-request>)))
  "Returns full string definition for message of type '<SupportLegs_Status-request>"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%int8 FootSupport_CMD~%int8 SUPPORT_DOUBLE=0~%int8 SUPPORT_SINGLE_RIGHT=1~%int8 SUPPORT_SINGLE_LEFT=2~%int8 GET_SUPPORT_STATUS=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SupportLegs_Status-request)))
  "Returns full string definition for message of type 'SupportLegs_Status-request"
  (cl:format cl:nil "~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%~%int8 FootSupport_CMD~%int8 SUPPORT_DOUBLE=0~%int8 SUPPORT_SINGLE_RIGHT=1~%int8 SUPPORT_SINGLE_LEFT=2~%int8 GET_SUPPORT_STATUS=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SupportLegs_Status-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SupportLegs_Status-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SupportLegs_Status-request
    (cl:cons ':FootSupport_CMD (FootSupport_CMD msg))
))
;//! \htmlinclude SupportLegs_Status-response.msg.html

(cl:defclass <SupportLegs_Status-response> (roslisp-msg-protocol:ros-message)
  ((FootSupport_Status
    :reader FootSupport_Status
    :initarg :FootSupport_Status
    :type cl:integer
    :initform 0))
)

(cl:defclass SupportLegs_Status-response (<SupportLegs_Status-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SupportLegs_Status-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SupportLegs_Status-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name hrl_kinematics-srv:<SupportLegs_Status-response> is deprecated: use hrl_kinematics-srv:SupportLegs_Status-response instead.")))

(cl:ensure-generic-function 'FootSupport_Status-val :lambda-list '(m))
(cl:defmethod FootSupport_Status-val ((m <SupportLegs_Status-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader hrl_kinematics-srv:FootSupport_Status-val is deprecated.  Use hrl_kinematics-srv:FootSupport_Status instead.")
  (FootSupport_Status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SupportLegs_Status-response>) ostream)
  "Serializes a message object of type '<SupportLegs_Status-response>"
  (cl:let* ((signed (cl:slot-value msg 'FootSupport_Status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SupportLegs_Status-response>) istream)
  "Deserializes a message object of type '<SupportLegs_Status-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'FootSupport_Status) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SupportLegs_Status-response>)))
  "Returns string type for a service object of type '<SupportLegs_Status-response>"
  "hrl_kinematics/SupportLegs_StatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SupportLegs_Status-response)))
  "Returns string type for a service object of type 'SupportLegs_Status-response"
  "hrl_kinematics/SupportLegs_StatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SupportLegs_Status-response>)))
  "Returns md5sum for a message object of type '<SupportLegs_Status-response>"
  "ec7194f86cdbf88898be0a72e8be6e9f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SupportLegs_Status-response)))
  "Returns md5sum for a message object of type 'SupportLegs_Status-response"
  "ec7194f86cdbf88898be0a72e8be6e9f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SupportLegs_Status-response>)))
  "Returns full string definition for message of type '<SupportLegs_Status-response>"
  (cl:format cl:nil "int32 FootSupport_Status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SupportLegs_Status-response)))
  "Returns full string definition for message of type 'SupportLegs_Status-response"
  (cl:format cl:nil "int32 FootSupport_Status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SupportLegs_Status-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SupportLegs_Status-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SupportLegs_Status-response
    (cl:cons ':FootSupport_Status (FootSupport_Status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SupportLegs_Status)))
  'SupportLegs_Status-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SupportLegs_Status)))
  'SupportLegs_Status-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SupportLegs_Status)))
  "Returns string type for a service object of type '<SupportLegs_Status>"
  "hrl_kinematics/SupportLegs_Status")