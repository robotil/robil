; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C23C11_OSM.msg.html

(cl:defclass <C23C11_OSM> (roslisp-msg-protocol:ros-message)
  ((ID
    :reader ID
    :initarg :ID
    :type cl:fixnum
    :initform 0)
   (NAME
    :reader NAME
    :initarg :NAME
    :type cl:string
    :initform "")
   (EXTR_3D
    :reader EXTR_3D
    :initarg :EXTR_3D
    :type C11_Agent-msg:coord
    :initform (cl:make-instance 'C11_Agent-msg:coord))
   (CNTR
    :reader CNTR
    :initarg :CNTR
    :type C11_Agent-msg:coord
    :initform (cl:make-instance 'C11_Agent-msg:coord))
   (ORIN
    :reader ORIN
    :initarg :ORIN
    :type C11_Agent-msg:D3SPACE
    :initform (cl:make-instance 'C11_Agent-msg:D3SPACE)))
)

(cl:defclass C23C11_OSM (<C23C11_OSM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C23C11_OSM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C23C11_OSM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C23C11_OSM> is deprecated: use C11_Agent-msg:C23C11_OSM instead.")))

(cl:ensure-generic-function 'ID-val :lambda-list '(m))
(cl:defmethod ID-val ((m <C23C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ID-val is deprecated.  Use C11_Agent-msg:ID instead.")
  (ID m))

(cl:ensure-generic-function 'NAME-val :lambda-list '(m))
(cl:defmethod NAME-val ((m <C23C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:NAME-val is deprecated.  Use C11_Agent-msg:NAME instead.")
  (NAME m))

(cl:ensure-generic-function 'EXTR_3D-val :lambda-list '(m))
(cl:defmethod EXTR_3D-val ((m <C23C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:EXTR_3D-val is deprecated.  Use C11_Agent-msg:EXTR_3D instead.")
  (EXTR_3D m))

(cl:ensure-generic-function 'CNTR-val :lambda-list '(m))
(cl:defmethod CNTR-val ((m <C23C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:CNTR-val is deprecated.  Use C11_Agent-msg:CNTR instead.")
  (CNTR m))

(cl:ensure-generic-function 'ORIN-val :lambda-list '(m))
(cl:defmethod ORIN-val ((m <C23C11_OSM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:ORIN-val is deprecated.  Use C11_Agent-msg:ORIN instead.")
  (ORIN m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C23C11_OSM>) ostream)
  "Serializes a message object of type '<C23C11_OSM>"
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
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'EXTR_3D) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'CNTR) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ORIN) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C23C11_OSM>) istream)
  "Deserializes a message object of type '<C23C11_OSM>"
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
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'EXTR_3D) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'CNTR) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ORIN) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C23C11_OSM>)))
  "Returns string type for a message object of type '<C23C11_OSM>"
  "C11_Agent/C23C11_OSM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23C11_OSM)))
  "Returns string type for a message object of type 'C23C11_OSM"
  "C11_Agent/C23C11_OSM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C23C11_OSM>)))
  "Returns md5sum for a message object of type '<C23C11_OSM>"
  "8b059a4c6d59e8652df8b326041d42f5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C23C11_OSM)))
  "Returns md5sum for a message object of type 'C23C11_OSM"
  "8b059a4c6d59e8652df8b326041d42f5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C23C11_OSM>)))
  "Returns full string definition for message of type '<C23C11_OSM>"
  (cl:format cl:nil "int16 ID~%string NAME~%C11_Agent/coord EXTR_3D~%C11_Agent/coord CNTR~%C11_Agent/D3SPACE ORIN~%================================================================================~%MSG: C11_Agent/coord~%float64 X~%float64 Y~%float64 Z~%================================================================================~%MSG: C11_Agent/D3SPACE~%float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C23C11_OSM)))
  "Returns full string definition for message of type 'C23C11_OSM"
  (cl:format cl:nil "int16 ID~%string NAME~%C11_Agent/coord EXTR_3D~%C11_Agent/coord CNTR~%C11_Agent/D3SPACE ORIN~%================================================================================~%MSG: C11_Agent/coord~%float64 X~%float64 Y~%float64 Z~%================================================================================~%MSG: C11_Agent/D3SPACE~%float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C23C11_OSM>))
  (cl:+ 0
     2
     4 (cl:length (cl:slot-value msg 'NAME))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'EXTR_3D))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'CNTR))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ORIN))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C23C11_OSM>))
  "Converts a ROS message object to a list"
  (cl:list 'C23C11_OSM
    (cl:cons ':ID (ID msg))
    (cl:cons ':NAME (NAME msg))
    (cl:cons ':EXTR_3D (EXTR_3D msg))
    (cl:cons ':CNTR (CNTR msg))
    (cl:cons ':ORIN (ORIN msg))
))
