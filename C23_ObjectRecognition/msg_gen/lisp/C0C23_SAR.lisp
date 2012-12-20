; Auto-generated. Do not edit!


(cl:in-package C23_ObjectRecognition-msg)


;//! \htmlinclude C0C23_SAR.msg.html

(cl:defclass <C0C23_SAR> (roslisp-msg-protocol:ros-message)
  ((SARV_L
    :reader SARV_L
    :initarg :SARV_L
    :type cl:float
    :initform 0.0)
   (SARV_U
    :reader SARV_U
    :initarg :SARV_U
    :type cl:float
    :initform 0.0)
   (SARH_L
    :reader SARH_L
    :initarg :SARH_L
    :type cl:float
    :initform 0.0)
   (SARH_R
    :reader SARH_R
    :initarg :SARH_R
    :type cl:float
    :initform 0.0))
)

(cl:defclass C0C23_SAR (<C0C23_SAR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C23_SAR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C23_SAR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-msg:<C0C23_SAR> is deprecated: use C23_ObjectRecognition-msg:C0C23_SAR instead.")))

(cl:ensure-generic-function 'SARV_L-val :lambda-list '(m))
(cl:defmethod SARV_L-val ((m <C0C23_SAR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:SARV_L-val is deprecated.  Use C23_ObjectRecognition-msg:SARV_L instead.")
  (SARV_L m))

(cl:ensure-generic-function 'SARV_U-val :lambda-list '(m))
(cl:defmethod SARV_U-val ((m <C0C23_SAR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:SARV_U-val is deprecated.  Use C23_ObjectRecognition-msg:SARV_U instead.")
  (SARV_U m))

(cl:ensure-generic-function 'SARH_L-val :lambda-list '(m))
(cl:defmethod SARH_L-val ((m <C0C23_SAR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:SARH_L-val is deprecated.  Use C23_ObjectRecognition-msg:SARH_L instead.")
  (SARH_L m))

(cl:ensure-generic-function 'SARH_R-val :lambda-list '(m))
(cl:defmethod SARH_R-val ((m <C0C23_SAR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:SARH_R-val is deprecated.  Use C23_ObjectRecognition-msg:SARH_R instead.")
  (SARH_R m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C23_SAR>) ostream)
  "Serializes a message object of type '<C0C23_SAR>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'SARV_L))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'SARV_U))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'SARH_L))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'SARH_R))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C23_SAR>) istream)
  "Deserializes a message object of type '<C0C23_SAR>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'SARV_L) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'SARV_U) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'SARH_L) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'SARH_R) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C23_SAR>)))
  "Returns string type for a message object of type '<C0C23_SAR>"
  "C23_ObjectRecognition/C0C23_SAR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C23_SAR)))
  "Returns string type for a message object of type 'C0C23_SAR"
  "C23_ObjectRecognition/C0C23_SAR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C23_SAR>)))
  "Returns md5sum for a message object of type '<C0C23_SAR>"
  "19b998d5d73668f9b2501d3193d623d5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C23_SAR)))
  "Returns md5sum for a message object of type 'C0C23_SAR"
  "19b998d5d73668f9b2501d3193d623d5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C23_SAR>)))
  "Returns full string definition for message of type '<C0C23_SAR>"
  (cl:format cl:nil "float32 SARV_L~%float32 SARV_U~%float32 SARH_L~%float32 SARH_R~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C23_SAR)))
  "Returns full string definition for message of type 'C0C23_SAR"
  (cl:format cl:nil "float32 SARV_L~%float32 SARV_U~%float32 SARH_L~%float32 SARH_R~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C23_SAR>))
  (cl:+ 0
     4
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C23_SAR>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C23_SAR
    (cl:cons ':SARV_L (SARV_L msg))
    (cl:cons ':SARV_U (SARV_U msg))
    (cl:cons ':SARH_L (SARH_L msg))
    (cl:cons ':SARH_R (SARH_R msg))
))
