; Auto-generated. Do not edit!


(cl:in-package C51_CarOperation-msg)


;//! \htmlinclude C0C51_TRA.msg.html

(cl:defclass <C0C51_TRA> (roslisp-msg-protocol:ros-message)
  ((TRA_RNDF
    :reader TRA_RNDF
    :initarg :TRA_RNDF
    :type cl:string
    :initform "")
   (TAR_MDF
    :reader TAR_MDF
    :initarg :TAR_MDF
    :type cl:string
    :initform "")
   (TAR_DES
    :reader TAR_DES
    :initarg :TAR_DES
    :type cl:string
    :initform ""))
)

(cl:defclass C0C51_TRA (<C0C51_TRA>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C51_TRA>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C51_TRA)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C51_CarOperation-msg:<C0C51_TRA> is deprecated: use C51_CarOperation-msg:C0C51_TRA instead.")))

(cl:ensure-generic-function 'TRA_RNDF-val :lambda-list '(m))
(cl:defmethod TRA_RNDF-val ((m <C0C51_TRA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:TRA_RNDF-val is deprecated.  Use C51_CarOperation-msg:TRA_RNDF instead.")
  (TRA_RNDF m))

(cl:ensure-generic-function 'TAR_MDF-val :lambda-list '(m))
(cl:defmethod TAR_MDF-val ((m <C0C51_TRA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:TAR_MDF-val is deprecated.  Use C51_CarOperation-msg:TAR_MDF instead.")
  (TAR_MDF m))

(cl:ensure-generic-function 'TAR_DES-val :lambda-list '(m))
(cl:defmethod TAR_DES-val ((m <C0C51_TRA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:TAR_DES-val is deprecated.  Use C51_CarOperation-msg:TAR_DES instead.")
  (TAR_DES m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C51_TRA>) ostream)
  "Serializes a message object of type '<C0C51_TRA>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'TRA_RNDF))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'TRA_RNDF))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'TAR_MDF))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'TAR_MDF))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'TAR_DES))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'TAR_DES))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C51_TRA>) istream)
  "Deserializes a message object of type '<C0C51_TRA>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TRA_RNDF) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'TRA_RNDF) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TAR_MDF) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'TAR_MDF) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'TAR_DES) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'TAR_DES) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C51_TRA>)))
  "Returns string type for a message object of type '<C0C51_TRA>"
  "C51_CarOperation/C0C51_TRA")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C51_TRA)))
  "Returns string type for a message object of type 'C0C51_TRA"
  "C51_CarOperation/C0C51_TRA")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C51_TRA>)))
  "Returns md5sum for a message object of type '<C0C51_TRA>"
  "16d4b6bf19408bb80fd1298061bb06e5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C51_TRA)))
  "Returns md5sum for a message object of type 'C0C51_TRA"
  "16d4b6bf19408bb80fd1298061bb06e5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C51_TRA>)))
  "Returns full string definition for message of type '<C0C51_TRA>"
  (cl:format cl:nil "string TRA_RNDF~%string TAR_MDF~%string TAR_DES~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C51_TRA)))
  "Returns full string definition for message of type 'C0C51_TRA"
  (cl:format cl:nil "string TRA_RNDF~%string TAR_MDF~%string TAR_DES~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C51_TRA>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'TRA_RNDF))
     4 (cl:length (cl:slot-value msg 'TAR_MDF))
     4 (cl:length (cl:slot-value msg 'TAR_DES))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C51_TRA>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C51_TRA
    (cl:cons ':TRA_RNDF (TRA_RNDF msg))
    (cl:cons ':TAR_MDF (TAR_MDF msg))
    (cl:cons ':TAR_DES (TAR_DES msg))
))
