; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C24C11_OSM.msg.html

(cl:defclass <C24C11_OSM> (roslisp-msg-protocol:ros-message)
  ((TYPE
    :reader TYPE
    :initarg :TYPE
    :type cl:integer
    :initform 0)
   (ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (COM
    :reader COM
    :initarg :COM
    :type (cl:vector C11_Agent-msg:coord)
   :initform (cl:make-array 0 :element-type 'C11_Agent-msg:coord :initial-element (cl:make-instance 'C11_Agent-msg:coord)))
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
   (EXTR_3D
    :reader EXTR_3D
    :initarg :EXTR_3D
    :type C11_Agent-msg:coord
    :initform (cl:make-instance 'C11_Agent-msg:coord)))
)

(cl:defclass C24C11_OSM (<C24C11_OSM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C24C11_OSM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C24C11_OSM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C24C11_OSM> is deprecated: use C11_Agent-msg:C24C11_OSM instead.")))

(cl:ensure-generic-function 'TYPE-val :lambda-list '(m))
(cl:defmethod TYPE-val ((m <C24C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:TYPE-val is deprecated.  Use C11_Agent-msg:TYPE instead.")
  (TYPE m))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <C24C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ID-val is deprecated.  Use C11_Agent-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'COM-val :lambda-list '(m))
(cl:defmethod COM-val ((m <C24C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:COM-val is deprecated.  Use C11_Agent-msg:COM instead.")
  (COM m))

(cl:ensure-generic-function 'EXTR_VIS-val :lambda-list '(m))
(cl:defmethod EXTR_VIS-val ((m <C24C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:EXTR_VIS-val is deprecated.  Use C11_Agent-msg:EXTR_VIS instead.")
  (EXTR_VIS m))

(cl:ensure-generic-function 'EXTR_TOP-val :lambda-list '(m))
(cl:defmethod EXTR_TOP-val ((m <C24C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:EXTR_TOP-val is deprecated.  Use C11_Agent-msg:EXTR_TOP instead.")
  (EXTR_TOP m))

(cl:ensure-generic-function 'EXTR_3D-val :lambda-list '(m))
(cl:defmethod EXTR_3D-val ((m <C24C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:EXTR_3D-val is deprecated.  Use C11_Agent-msg:EXTR_3D instead.")
  (EXTR_3D m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C24C11_OSM>)))
    "Constants for message type '<C24C11_OSM>"
  '((:TYPE_STATIC . 0)
    (:TYPE_DYNAMIC . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C24C11_OSM)))
    "Constants for message type 'C24C11_OSM"
  '((:TYPE_STATIC . 0)
    (:TYPE_DYNAMIC . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C24C11_OSM>) ostream)
  "Serializes a message object of type '<C24C11_OSM>"
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
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'COM))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'COM))
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'EXTR_3D) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C24C11_OSM>) istream)
  "Deserializes a message object of type '<C24C11_OSM>"
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
  (cl:setf (cl:slot-value msg 'COM) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'COM)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'C11_Agent-msg:coord))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'EXTR_3D) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C24C11_OSM>)))
  "Returns string type for a message object of type '<C24C11_OSM>"
  "C11_Agent/C24C11_OSM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C24C11_OSM)))
  "Returns string type for a message object of type 'C24C11_OSM"
  "C11_Agent/C24C11_OSM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C24C11_OSM>)))
  "Returns md5sum for a message object of type '<C24C11_OSM>"
  "9bfebbc3c0618ea996554dd91c71e678")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C24C11_OSM)))
  "Returns md5sum for a message object of type 'C24C11_OSM"
  "9bfebbc3c0618ea996554dd91c71e678")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C24C11_OSM>)))
  "Returns full string definition for message of type '<C24C11_OSM>"
  (cl:format cl:nil "int32 TYPE~%int32 TYPE_STATIC  = 0~%int32 TYPE_DYNAMIC = 1~%int16 ID~%C11_Agent/coord[] COM~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/coord EXTR_3D~%================================================================================~%MSG: C11_Agent/coord~%float64 X~%float64 Y~%float64 Z~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C24C11_OSM)))
  "Returns full string definition for message of type 'C24C11_OSM"
  (cl:format cl:nil "int32 TYPE~%int32 TYPE_STATIC  = 0~%int32 TYPE_DYNAMIC = 1~%int16 ID~%C11_Agent/coord[] COM~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/coord EXTR_3D~%================================================================================~%MSG: C11_Agent/coord~%float64 X~%float64 Y~%float64 Z~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C24C11_OSM>))
  (cl:+ 0
     4
     2
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'COM) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'EXTR_VIS) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'EXTR_TOP) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'EXTR_3D))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C24C11_OSM>))
  "Converts a ROS message object to a list"
  (cl:list 'C24C11_OSM
    (cl:cons ':TYPE (TYPE msg))
    (cl:cons ':ID (ID msg))
    (cl:cons ':COM (COM msg))
    (cl:cons ':EXTR_VIS (EXTR_VIS msg))
    (cl:cons ':EXTR_TOP (EXTR_TOP msg))
    (cl:cons ':EXTR_3D (EXTR_3D msg))
))
