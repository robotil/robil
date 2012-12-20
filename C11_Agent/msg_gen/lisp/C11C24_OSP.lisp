; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C11C24_OSP.msg.html

(cl:defclass <C11C24_OSP> (roslisp-msg-protocol:ros-message)
  ((ACT_TYPE
    :reader ACT_TYPE
    :initarg :ACT_TYPE
    :type cl:integer
    :initform 0)
   (FRZ
    :reader FRZ
    :initarg :FRZ
    :type cl:integer
    :initform 0)
   (TYPE
    :reader TYPE
    :initarg :TYPE
    :type cl:integer
    :initform 0)
   (ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (EXTR_VIS
    :reader EXTR_VIS
    :initarg :EXTR_VIS
    :type (cl:vector C11_Agent-msg:pathLocation)
   :initform (cl:make-array 0 :element-type 'C11_Agent-msg:pathLocation :initial-element (cl:make-instance 'C11_Agent-msg:pathLocation)))
   (EXTR_TOP
    :reader EXTR_TOP
    :initarg :EXTR_TOP
    :type (cl:vector C11_Agent-msg:pathLocation)
   :initform (cl:make-array 0 :element-type 'C11_Agent-msg:pathLocation :initial-element (cl:make-instance 'C11_Agent-msg:pathLocation)))
   (IGNR
    :reader IGNR
    :initarg :IGNR
    :type (cl:vector C11_Agent-msg:pathLocation)
   :initform (cl:make-array 0 :element-type 'C11_Agent-msg:pathLocation :initial-element (cl:make-instance 'C11_Agent-msg:pathLocation))))
)

(cl:defclass C11C24_OSP (<C11C24_OSP>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11C24_OSP>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11C24_OSP)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C11C24_OSP> is deprecated: use C11_Agent-msg:C11C24_OSP instead.")))

(cl:ensure-generic-function 'ACT_TYPE-val :lambda-list '(m))
(cl:defmethod ACT_TYPE-val ((m <C11C24_OSP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ACT_TYPE-val is deprecated.  Use C11_Agent-msg:ACT_TYPE instead.")
  (ACT_TYPE m))

(cl:ensure-generic-function 'FRZ-val :lambda-list '(m))
(cl:defmethod FRZ-val ((m <C11C24_OSP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:FRZ-val is deprecated.  Use C11_Agent-msg:FRZ instead.")
  (FRZ m))

(cl:ensure-generic-function 'TYPE-val :lambda-list '(m))
(cl:defmethod TYPE-val ((m <C11C24_OSP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:TYPE-val is deprecated.  Use C11_Agent-msg:TYPE instead.")
  (TYPE m))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <C11C24_OSP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ID-val is deprecated.  Use C11_Agent-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'EXTR_VIS-val :lambda-list '(m))
(cl:defmethod EXTR_VIS-val ((m <C11C24_OSP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:EXTR_VIS-val is deprecated.  Use C11_Agent-msg:EXTR_VIS instead.")
  (EXTR_VIS m))

(cl:ensure-generic-function 'EXTR_TOP-val :lambda-list '(m))
(cl:defmethod EXTR_TOP-val ((m <C11C24_OSP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:EXTR_TOP-val is deprecated.  Use C11_Agent-msg:EXTR_TOP instead.")
  (EXTR_TOP m))

(cl:ensure-generic-function 'IGNR-val :lambda-list '(m))
(cl:defmethod IGNR-val ((m <C11C24_OSP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:IGNR-val is deprecated.  Use C11_Agent-msg:IGNR instead.")
  (IGNR m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C11C24_OSP>)))
    "Constants for message type '<C11C24_OSP>"
  '((:ACT_TYPE_MODIFIED . 0)
    (:ACT_TYPE_DELETED . 1)
    (:ACT_TYPE_NEW . 2)
    (:FRZ_KEEP . 0)
    (:FRZ_RETRY . 1)
    (:TYPE_STATIC . 0)
    (:TYPE_DYNAMIC . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C11C24_OSP)))
    "Constants for message type 'C11C24_OSP"
  '((:ACT_TYPE_MODIFIED . 0)
    (:ACT_TYPE_DELETED . 1)
    (:ACT_TYPE_NEW . 2)
    (:FRZ_KEEP . 0)
    (:FRZ_RETRY . 1)
    (:TYPE_STATIC . 0)
    (:TYPE_DYNAMIC . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11C24_OSP>) ostream)
  "Serializes a message object of type '<C11C24_OSP>"
  (cl:let* ((signed (cl:slot-value msg 'ACT_TYPE)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'FRZ)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'TYPE)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'EXTR_VIS))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'EXTR_VIS))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'EXTR_TOP))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'EXTR_TOP))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'IGNR))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'IGNR))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11C24_OSP>) istream)
  "Deserializes a message object of type '<C11C24_OSP>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ACT_TYPE) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'FRZ) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TYPE) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'EXTR_VIS) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'EXTR_VIS)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'C11_Agent-msg:pathLocation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'EXTR_TOP) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'EXTR_TOP)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'C11_Agent-msg:pathLocation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'IGNR) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'IGNR)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'C11_Agent-msg:pathLocation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11C24_OSP>)))
  "Returns string type for a message object of type '<C11C24_OSP>"
  "C11_Agent/C11C24_OSP")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11C24_OSP)))
  "Returns string type for a message object of type 'C11C24_OSP"
  "C11_Agent/C11C24_OSP")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11C24_OSP>)))
  "Returns md5sum for a message object of type '<C11C24_OSP>"
  "37a469056ba4f48cc45e0bee050bf62f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11C24_OSP)))
  "Returns md5sum for a message object of type 'C11C24_OSP"
  "37a469056ba4f48cc45e0bee050bf62f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11C24_OSP>)))
  "Returns full string definition for message of type '<C11C24_OSP>"
  (cl:format cl:nil "int32 ACT_TYPE~%int32 ACT_TYPE_MODIFIED = 0~%int32 ACT_TYPE_DELETED  = 1~%int32 ACT_TYPE_NEW      = 2~%int32 FRZ~%int32 FRZ_KEEP  = 0~%int32 FRZ_RETRY = 1~%int32 TYPE~%int32 TYPE_STATIC=0~%int32 TYPE_DYNAMIC=1~%int16 ID~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/pathLocation[] IGNR~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11C24_OSP)))
  "Returns full string definition for message of type 'C11C24_OSP"
  (cl:format cl:nil "int32 ACT_TYPE~%int32 ACT_TYPE_MODIFIED = 0~%int32 ACT_TYPE_DELETED  = 1~%int32 ACT_TYPE_NEW      = 2~%int32 FRZ~%int32 FRZ_KEEP  = 0~%int32 FRZ_RETRY = 1~%int32 TYPE~%int32 TYPE_STATIC=0~%int32 TYPE_DYNAMIC=1~%int16 ID~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/pathLocation[] IGNR~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11C24_OSP>))
  (cl:+ 0
     4
     4
     4
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'EXTR_VIS) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'EXTR_TOP) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'IGNR) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11C24_OSP>))
  "Converts a ROS message object to a list"
  (cl:list 'C11C24_OSP
    (cl:cons ':ACT_TYPE (ACT_TYPE msg))
    (cl:cons ':FRZ (FRZ msg))
    (cl:cons ':TYPE (TYPE msg))
    (cl:cons ':ID (ID msg))
    (cl:cons ':EXTR_VIS (EXTR_VIS msg))
    (cl:cons ':EXTR_TOP (EXTR_TOP msg))
    (cl:cons ':IGNR (IGNR msg))
))
