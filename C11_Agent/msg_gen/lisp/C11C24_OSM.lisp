; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C11C24_OSM.msg.html

(cl:defclass <C11C24_OSM> (roslisp-msg-protocol:ros-message)
  ((TYP
    :reader TYP
    :initarg :TYP
    :type cl:fixnum
    :initform 0)
   (LAT
    :reader LAT
    :initarg :LAT
    :type cl:float
    :initform 0.0)
   (SCN
    :reader SCN
    :initarg :SCN
    :type cl:fixnum
    :initform 0)
   (MOV
    :reader MOV
    :initarg :MOV
    :type cl:fixnum
    :initform 0))
)

(cl:defclass C11C24_OSM (<C11C24_OSM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11C24_OSM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11C24_OSM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C11C24_OSM> is deprecated: use C11_Agent-msg:C11C24_OSM instead.")))

(cl:ensure-generic-function 'TYP-val :lambda-list '(m))
(cl:defmethod TYP-val ((m <C11C24_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:TYP-val is deprecated.  Use C11_Agent-msg:TYP instead.")
  (TYP m))

(cl:ensure-generic-function 'LAT-val :lambda-list '(m))
(cl:defmethod LAT-val ((m <C11C24_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:LAT-val is deprecated.  Use C11_Agent-msg:LAT instead.")
  (LAT m))

(cl:ensure-generic-function 'SCN-val :lambda-list '(m))
(cl:defmethod SCN-val ((m <C11C24_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:SCN-val is deprecated.  Use C11_Agent-msg:SCN instead.")
  (SCN m))

(cl:ensure-generic-function 'MOV-val :lambda-list '(m))
(cl:defmethod MOV-val ((m <C11C24_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:MOV-val is deprecated.  Use C11_Agent-msg:MOV instead.")
  (MOV m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C11C24_OSM>)))
    "Constants for message type '<C11C24_OSM>"
  '((:TYP_STATIC . 0)
    (:TYP_DYNAMIC . 1)
    (:SCN_SCAN . 0)
    (:SCN_CURRENT . 1)
    (:MOV_NONE . 0)
    (:MOV_HEAD . 1)
    (:MOV_POSTURE . 2)
    (:MOV_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C11C24_OSM)))
    "Constants for message type 'C11C24_OSM"
  '((:TYP_STATIC . 0)
    (:TYP_DYNAMIC . 1)
    (:SCN_SCAN . 0)
    (:SCN_CURRENT . 1)
    (:MOV_NONE . 0)
    (:MOV_HEAD . 1)
    (:MOV_POSTURE . 2)
    (:MOV_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11C24_OSM>) ostream)
  "Serializes a message object of type '<C11C24_OSM>"
  (cl:let* ((signed (cl:slot-value msg 'TYP)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'LAT))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'SCN)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'MOV)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11C24_OSM>) istream)
  "Deserializes a message object of type '<C11C24_OSM>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TYP) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'LAT) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'SCN) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'MOV) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11C24_OSM>)))
  "Returns string type for a message object of type '<C11C24_OSM>"
  "C11_Agent/C11C24_OSM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11C24_OSM)))
  "Returns string type for a message object of type 'C11C24_OSM"
  "C11_Agent/C11C24_OSM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11C24_OSM>)))
  "Returns md5sum for a message object of type '<C11C24_OSM>"
  "7db60021ed08c471a4cee4cf1a3157c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11C24_OSM)))
  "Returns md5sum for a message object of type 'C11C24_OSM"
  "7db60021ed08c471a4cee4cf1a3157c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11C24_OSM>)))
  "Returns full string definition for message of type '<C11C24_OSM>"
  (cl:format cl:nil "int16 TYP~%int16 TYP_STATIC  = 0~%int16 TYP_DYNAMIC = 1~%float64 LAT~%int16 SCN~%int16 SCN_SCAN    = 0~%int16 SCN_CURRENT = 1~%int16 MOV~%int16 MOV_NONE     = 0~%int16 MOV_HEAD     = 1~%int16 MOV_POSTURE  = 2~%int16 MOV_POSITION = 3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11C24_OSM)))
  "Returns full string definition for message of type 'C11C24_OSM"
  (cl:format cl:nil "int16 TYP~%int16 TYP_STATIC  = 0~%int16 TYP_DYNAMIC = 1~%float64 LAT~%int16 SCN~%int16 SCN_SCAN    = 0~%int16 SCN_CURRENT = 1~%int16 MOV~%int16 MOV_NONE     = 0~%int16 MOV_HEAD     = 1~%int16 MOV_POSTURE  = 2~%int16 MOV_POSITION = 3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11C24_OSM>))
  (cl:+ 0
     2
     8
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11C24_OSM>))
  "Converts a ROS message object to a list"
  (cl:list 'C11C24_OSM
    (cl:cons ':TYP (TYP msg))
    (cl:cons ':LAT (LAT msg))
    (cl:cons ':SCN (SCN msg))
    (cl:cons ':MOV (MOV msg))
))
