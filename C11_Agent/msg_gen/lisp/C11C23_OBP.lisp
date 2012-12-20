; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C11C23_OBP.msg.html

(cl:defclass <C11C23_OBP> (roslisp-msg-protocol:ros-message)
  ((ACT
    :reader ACT
    :initarg :ACT
    :type cl:fixnum
    :initform 0)
   (FRZ
    :reader FRZ
    :initarg :FRZ
    :type cl:fixnum
    :initform 0)
   (ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (NAME
    :reader NAME
    :initarg :NAME
    :type cl:string
    :initform "")
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
   (ORI
    :reader ORI
    :initarg :ORI
    :type C11_Agent-msg:D3SPACE
    :initform (cl:make-instance 'C11_Agent-msg:D3SPACE)))
)

(cl:defclass C11C23_OBP (<C11C23_OBP>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11C23_OBP>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11C23_OBP)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C11C23_OBP> is deprecated: use C11_Agent-msg:C11C23_OBP instead.")))

(cl:ensure-generic-function 'ACT-val :lambda-list '(m))
(cl:defmethod ACT-val ((m <C11C23_OBP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ACT-val is deprecated.  Use C11_Agent-msg:ACT instead.")
  (ACT m))

(cl:ensure-generic-function 'FRZ-val :lambda-list '(m))
(cl:defmethod FRZ-val ((m <C11C23_OBP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:FRZ-val is deprecated.  Use C11_Agent-msg:FRZ instead.")
  (FRZ m))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <C11C23_OBP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ID-val is deprecated.  Use C11_Agent-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'NAME-val :lambda-list '(m))
(cl:defmethod NAME-val ((m <C11C23_OBP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:NAME-val is deprecated.  Use C11_Agent-msg:NAME instead.")
  (NAME m))

(cl:ensure-generic-function 'EXTR_VIS-val :lambda-list '(m))
(cl:defmethod EXTR_VIS-val ((m <C11C23_OBP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:EXTR_VIS-val is deprecated.  Use C11_Agent-msg:EXTR_VIS instead.")
  (EXTR_VIS m))

(cl:ensure-generic-function 'EXTR_TOP-val :lambda-list '(m))
(cl:defmethod EXTR_TOP-val ((m <C11C23_OBP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:EXTR_TOP-val is deprecated.  Use C11_Agent-msg:EXTR_TOP instead.")
  (EXTR_TOP m))

(cl:ensure-generic-function 'ORI-val :lambda-list '(m))
(cl:defmethod ORI-val ((m <C11C23_OBP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ORI-val is deprecated.  Use C11_Agent-msg:ORI instead.")
  (ORI m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C11C23_OBP>)))
    "Constants for message type '<C11C23_OBP>"
  '((:ACT_MODIFIED . 0)
    (:ACT_NEW . 1)
    (:FRZ_KEEP . 0)
    (:ACT_RETRY . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C11C23_OBP)))
    "Constants for message type 'C11C23_OBP"
  '((:ACT_MODIFIED . 0)
    (:ACT_NEW . 1)
    (:FRZ_KEEP . 0)
    (:ACT_RETRY . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11C23_OBP>) ostream)
  "Serializes a message object of type '<C11C23_OBP>"
  (cl:let* ((signed (cl:slot-value msg 'ACT)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'FRZ)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'ID)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'NAME))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'NAME))
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ORI) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11C23_OBP>) istream)
  "Deserializes a message object of type '<C11C23_OBP>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ACT) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'FRZ) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ID) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'NAME) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'NAME) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ORI) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11C23_OBP>)))
  "Returns string type for a message object of type '<C11C23_OBP>"
  "C11_Agent/C11C23_OBP")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11C23_OBP)))
  "Returns string type for a message object of type 'C11C23_OBP"
  "C11_Agent/C11C23_OBP")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11C23_OBP>)))
  "Returns md5sum for a message object of type '<C11C23_OBP>"
  "ff2662649f64159b448d8453ec2a31d6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11C23_OBP)))
  "Returns md5sum for a message object of type 'C11C23_OBP"
  "ff2662649f64159b448d8453ec2a31d6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11C23_OBP>)))
  "Returns full string definition for message of type '<C11C23_OBP>"
  (cl:format cl:nil "int16 ACT~%int16 ACT_MODIFIED = 0~%int16 ACT_NEW      = 1~%int16 FRZ~%int16 FRZ_KEEP  = 0~%int16 ACT_RETRY = 1~%int16 ID~%string NAME~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/D3SPACE ORI~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%================================================================================~%MSG: C11_Agent/D3SPACE~%float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11C23_OBP)))
  "Returns full string definition for message of type 'C11C23_OBP"
  (cl:format cl:nil "int16 ACT~%int16 ACT_MODIFIED = 0~%int16 ACT_NEW      = 1~%int16 FRZ~%int16 FRZ_KEEP  = 0~%int16 ACT_RETRY = 1~%int16 ID~%string NAME~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/D3SPACE ORI~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%================================================================================~%MSG: C11_Agent/D3SPACE~%float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11C23_OBP>))
  (cl:+ 0
     2
     2
     2
     4 (cl:length (cl:slot-value msg 'NAME))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'EXTR_VIS) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'EXTR_TOP) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ORI))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11C23_OBP>))
  "Converts a ROS message object to a list"
  (cl:list 'C11C23_OBP
    (cl:cons ':ACT (ACT msg))
    (cl:cons ':FRZ (FRZ msg))
    (cl:cons ':ID (ID msg))
    (cl:cons ':NAME (NAME msg))
    (cl:cons ':EXTR_VIS (EXTR_VIS msg))
    (cl:cons ':EXTR_TOP (EXTR_TOP msg))
    (cl:cons ':ORI (ORI msg))
))
