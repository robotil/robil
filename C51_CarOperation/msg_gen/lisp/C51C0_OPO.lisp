; Auto-generated. Do not edit!


(cl:in-package C51_CarOperation-msg)


;//! \htmlinclude C51C0_OPO.msg.html

(cl:defclass <C51C0_OPO> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:integer
    :initform 0)
   (y
    :reader y
    :initarg :y
    :type cl:integer
    :initform 0)
   (z
    :reader z
    :initarg :z
    :type cl:integer
    :initform 0))
)

(cl:defclass C51C0_OPO (<C51C0_OPO>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C51C0_OPO>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C51C0_OPO)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C51_CarOperation-msg:<C51C0_OPO> is deprecated: use C51_CarOperation-msg:C51C0_OPO instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <C51C0_OPO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:x-val is deprecated.  Use C51_CarOperation-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <C51C0_OPO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:y-val is deprecated.  Use C51_CarOperation-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <C51C0_OPO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:z-val is deprecated.  Use C51_CarOperation-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C51C0_OPO>) ostream)
  "Serializes a message object of type '<C51C0_OPO>"
  (cl:let* ((signed (cl:slot-value msg 'x)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'y)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'z)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C51C0_OPO>) istream)
  "Deserializes a message object of type '<C51C0_OPO>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'x) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'y) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'z) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C51C0_OPO>)))
  "Returns string type for a message object of type '<C51C0_OPO>"
  "C51_CarOperation/C51C0_OPO")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C51C0_OPO)))
  "Returns string type for a message object of type 'C51C0_OPO"
  "C51_CarOperation/C51C0_OPO")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C51C0_OPO>)))
  "Returns md5sum for a message object of type '<C51C0_OPO>"
  "3cb41a2c4416de195dbb95b7777a06fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C51C0_OPO)))
  "Returns md5sum for a message object of type 'C51C0_OPO"
  "3cb41a2c4416de195dbb95b7777a06fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C51C0_OPO>)))
  "Returns full string definition for message of type '<C51C0_OPO>"
  (cl:format cl:nil "int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C51C0_OPO)))
  "Returns full string definition for message of type 'C51C0_OPO"
  (cl:format cl:nil "int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C51C0_OPO>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C51C0_OPO>))
  "Converts a ROS message object to a list"
  (cl:list 'C51C0_OPO
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
