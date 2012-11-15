; Auto-generated. Do not edit!


(cl:in-package C23_ObjectRecognition-msg)


;//! \htmlinclude TBD.msg.html

(cl:defclass <TBD> (roslisp-msg-protocol:ros-message)
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

(cl:defclass TBD (<TBD>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TBD>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TBD)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-msg:<TBD> is deprecated: use C23_ObjectRecognition-msg:TBD instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <TBD>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:x-val is deprecated.  Use C23_ObjectRecognition-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <TBD>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:y-val is deprecated.  Use C23_ObjectRecognition-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'z-val :lambda-list '(m))
(cl:defmethod z-val ((m <TBD>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:z-val is deprecated.  Use C23_ObjectRecognition-msg:z instead.")
  (z m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TBD>) ostream)
  "Serializes a message object of type '<TBD>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TBD>) istream)
  "Deserializes a message object of type '<TBD>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TBD>)))
  "Returns string type for a message object of type '<TBD>"
  "C23_ObjectRecognition/TBD")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TBD)))
  "Returns string type for a message object of type 'TBD"
  "C23_ObjectRecognition/TBD")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TBD>)))
  "Returns md5sum for a message object of type '<TBD>"
  "3cb41a2c4416de195dbb95b7777a06fb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TBD)))
  "Returns md5sum for a message object of type 'TBD"
  "3cb41a2c4416de195dbb95b7777a06fb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TBD>)))
  "Returns full string definition for message of type '<TBD>"
  (cl:format cl:nil "int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TBD)))
  "Returns full string definition for message of type 'TBD"
  (cl:format cl:nil "int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TBD>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TBD>))
  "Converts a ROS message object to a list"
  (cl:list 'TBD
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':z (z msg))
))
