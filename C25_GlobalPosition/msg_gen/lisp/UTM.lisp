; Auto-generated. Do not edit!


(cl:in-package C25_GlobalPosition-msg)


;//! \htmlinclude UTM.msg.html

(cl:defclass <UTM> (roslisp-msg-protocol:ros-message)
  ((zone
    :reader zone
    :initarg :zone
    :type cl:integer
    :initform 0)
   (easting
    :reader easting
    :initarg :easting
    :type cl:integer
    :initform 0)
   (northing
    :reader northing
    :initarg :northing
    :type cl:integer
    :initform 0)
   (hemisphere
    :reader hemisphere
    :initarg :hemisphere
    :type cl:integer
    :initform 0))
)

(cl:defclass UTM (<UTM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UTM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UTM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C25_GlobalPosition-msg:<UTM> is deprecated: use C25_GlobalPosition-msg:UTM instead.")))

(cl:ensure-generic-function 'zone-val :lambda-list '(m))
(cl:defmethod zone-val ((m <UTM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-msg:zone-val is deprecated.  Use C25_GlobalPosition-msg:zone instead.")
  (zone m))

(cl:ensure-generic-function 'easting-val :lambda-list '(m))
(cl:defmethod easting-val ((m <UTM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-msg:easting-val is deprecated.  Use C25_GlobalPosition-msg:easting instead.")
  (easting m))

(cl:ensure-generic-function 'northing-val :lambda-list '(m))
(cl:defmethod northing-val ((m <UTM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-msg:northing-val is deprecated.  Use C25_GlobalPosition-msg:northing instead.")
  (northing m))

(cl:ensure-generic-function 'hemisphere-val :lambda-list '(m))
(cl:defmethod hemisphere-val ((m <UTM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-msg:hemisphere-val is deprecated.  Use C25_GlobalPosition-msg:hemisphere instead.")
  (hemisphere m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<UTM>)))
    "Constants for message type '<UTM>"
  '((:NORTHERN_HEMISPHERE . 0)
    (:SOUTHERN_HEMISPHERE . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'UTM)))
    "Constants for message type 'UTM"
  '((:NORTHERN_HEMISPHERE . 0)
    (:SOUTHERN_HEMISPHERE . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UTM>) ostream)
  "Serializes a message object of type '<UTM>"
  (cl:let* ((signed (cl:slot-value msg 'zone)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'easting)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'northing)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'hemisphere)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UTM>) istream)
  "Deserializes a message object of type '<UTM>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'zone) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'easting) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'northing) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'hemisphere) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UTM>)))
  "Returns string type for a message object of type '<UTM>"
  "C25_GlobalPosition/UTM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UTM)))
  "Returns string type for a message object of type 'UTM"
  "C25_GlobalPosition/UTM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UTM>)))
  "Returns md5sum for a message object of type '<UTM>"
  "1bca048ba614aa29513b05a11622a3ac")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UTM)))
  "Returns md5sum for a message object of type 'UTM"
  "1bca048ba614aa29513b05a11622a3ac")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UTM>)))
  "Returns full string definition for message of type '<UTM>"
  (cl:format cl:nil "int32 zone~%int64 easting~%int64 northing~%int32 hemisphere~%int32 NORTHERN_HEMISPHERE=0~%int32 SOUTHERN_HEMISPHERE=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UTM)))
  "Returns full string definition for message of type 'UTM"
  (cl:format cl:nil "int32 zone~%int64 easting~%int64 northing~%int32 hemisphere~%int32 NORTHERN_HEMISPHERE=0~%int32 SOUTHERN_HEMISPHERE=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UTM>))
  (cl:+ 0
     4
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UTM>))
  "Converts a ROS message object to a list"
  (cl:list 'UTM
    (cl:cons ':zone (zone msg))
    (cl:cons ':easting (easting msg))
    (cl:cons ':northing (northing msg))
    (cl:cons ':hemisphere (hemisphere msg))
))
