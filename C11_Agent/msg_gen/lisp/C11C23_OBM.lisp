; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C11C23_OBM.msg.html

(cl:defclass <C11C23_OBM> (roslisp-msg-protocol:ros-message)
  ((LAT
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

(cl:defclass C11C23_OBM (<C11C23_OBM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11C23_OBM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11C23_OBM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C11C23_OBM> is deprecated: use C11_Agent-msg:C11C23_OBM instead.")))

(cl:ensure-generic-function 'LAT-val :lambda-list '(m))
(cl:defmethod LAT-val ((m <C11C23_OBM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:LAT-val is deprecated.  Use C11_Agent-msg:LAT instead.")
  (LAT m))

(cl:ensure-generic-function 'SCN-val :lambda-list '(m))
(cl:defmethod SCN-val ((m <C11C23_OBM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:SCN-val is deprecated.  Use C11_Agent-msg:SCN instead.")
  (SCN m))

(cl:ensure-generic-function 'MOV-val :lambda-list '(m))
(cl:defmethod MOV-val ((m <C11C23_OBM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:MOV-val is deprecated.  Use C11_Agent-msg:MOV instead.")
  (MOV m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C11C23_OBM>)))
    "Constants for message type '<C11C23_OBM>"
  '((:SCN_SCAN . 0)
    (:SCN_CURRENT . 1)
    (:MOV_NONE . 0)
    (:MOV_HEAD . 1)
    (:MOV_POSTURE . 2)
    (:MOV_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C11C23_OBM)))
    "Constants for message type 'C11C23_OBM"
  '((:SCN_SCAN . 0)
    (:SCN_CURRENT . 1)
    (:MOV_NONE . 0)
    (:MOV_HEAD . 1)
    (:MOV_POSTURE . 2)
    (:MOV_POSITION . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11C23_OBM>) ostream)
  "Serializes a message object of type '<C11C23_OBM>"
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11C23_OBM>) istream)
  "Deserializes a message object of type '<C11C23_OBM>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11C23_OBM>)))
  "Returns string type for a message object of type '<C11C23_OBM>"
  "C11_Agent/C11C23_OBM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11C23_OBM)))
  "Returns string type for a message object of type 'C11C23_OBM"
  "C11_Agent/C11C23_OBM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11C23_OBM>)))
  "Returns md5sum for a message object of type '<C11C23_OBM>"
  "021b2b252d90f9fb11b61af4d8bd8a44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11C23_OBM)))
  "Returns md5sum for a message object of type 'C11C23_OBM"
  "021b2b252d90f9fb11b61af4d8bd8a44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11C23_OBM>)))
  "Returns full string definition for message of type '<C11C23_OBM>"
  (cl:format cl:nil "float64 LAT~%int16 SCN~%int16 SCN_SCAN=0~%int16 SCN_CURRENT=1~%int16 MOV~%int16 MOV_NONE     = 0~%int16 MOV_HEAD     = 1~%int16 MOV_POSTURE  = 2~%int16 MOV_POSITION = 3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11C23_OBM)))
  "Returns full string definition for message of type 'C11C23_OBM"
  (cl:format cl:nil "float64 LAT~%int16 SCN~%int16 SCN_SCAN=0~%int16 SCN_CURRENT=1~%int16 MOV~%int16 MOV_NONE     = 0~%int16 MOV_HEAD     = 1~%int16 MOV_POSTURE  = 2~%int16 MOV_POSITION = 3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11C23_OBM>))
  (cl:+ 0
     8
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11C23_OBM>))
  "Converts a ROS message object to a list"
  (cl:list 'C11C23_OBM
    (cl:cons ':LAT (LAT msg))
    (cl:cons ':SCN (SCN msg))
    (cl:cons ':MOV (MOV msg))
))
