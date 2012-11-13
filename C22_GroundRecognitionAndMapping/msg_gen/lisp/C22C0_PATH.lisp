; Auto-generated. Do not edit!


(cl:in-package C22_GroundRecognitionAndMapping-msg)


;//! \htmlinclude C22C0_PATH.msg.html

(cl:defclass <C22C0_PATH> (roslisp-msg-protocol:ros-message)
  ((PATH_Left
    :reader PATH_Left
    :initarg :PATH_Left
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (PATH_Right
    :reader PATH_Right
    :initarg :PATH_Right
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (PATH_Slope
    :reader PATH_Slope
    :initarg :PATH_Slope
    :type cl:float
    :initform 0.0)
   (PATH_ROU
    :reader PATH_ROU
    :initarg :PATH_ROU
    :type cl:integer
    :initform 0))
)

(cl:defclass C22C0_PATH (<C22C0_PATH>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C22C0_PATH>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C22C0_PATH)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C22_GroundRecognitionAndMapping-msg:<C22C0_PATH> is deprecated: use C22_GroundRecognitionAndMapping-msg:C22C0_PATH instead.")))

(cl:ensure-generic-function 'PATH_Left-val :lambda-list '(m))
(cl:defmethod PATH_Left-val ((m <C22C0_PATH>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-msg:PATH_Left-val is deprecated.  Use C22_GroundRecognitionAndMapping-msg:PATH_Left instead.")
  (PATH_Left m))

(cl:ensure-generic-function 'PATH_Right-val :lambda-list '(m))
(cl:defmethod PATH_Right-val ((m <C22C0_PATH>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-msg:PATH_Right-val is deprecated.  Use C22_GroundRecognitionAndMapping-msg:PATH_Right instead.")
  (PATH_Right m))

(cl:ensure-generic-function 'PATH_Slope-val :lambda-list '(m))
(cl:defmethod PATH_Slope-val ((m <C22C0_PATH>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-msg:PATH_Slope-val is deprecated.  Use C22_GroundRecognitionAndMapping-msg:PATH_Slope instead.")
  (PATH_Slope m))

(cl:ensure-generic-function 'PATH_ROU-val :lambda-list '(m))
(cl:defmethod PATH_ROU-val ((m <C22C0_PATH>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-msg:PATH_ROU-val is deprecated.  Use C22_GroundRecognitionAndMapping-msg:PATH_ROU instead.")
  (PATH_ROU m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C22C0_PATH>) ostream)
  "Serializes a message object of type '<C22C0_PATH>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PATH_Left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'PATH_Left))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PATH_Right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'PATH_Right))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PATH_Slope))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'PATH_ROU)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C22C0_PATH>) istream)
  "Deserializes a message object of type '<C22C0_PATH>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'PATH_Left) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PATH_Left)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'PATH_Right) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PATH_Right)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PATH_Slope) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'PATH_ROU) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C22C0_PATH>)))
  "Returns string type for a message object of type '<C22C0_PATH>"
  "C22_GroundRecognitionAndMapping/C22C0_PATH")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C22C0_PATH)))
  "Returns string type for a message object of type 'C22C0_PATH"
  "C22_GroundRecognitionAndMapping/C22C0_PATH")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C22C0_PATH>)))
  "Returns md5sum for a message object of type '<C22C0_PATH>"
  "78d5bbd55733684e8696339798d3f8ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C22C0_PATH)))
  "Returns md5sum for a message object of type 'C22C0_PATH"
  "78d5bbd55733684e8696339798d3f8ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C22C0_PATH>)))
  "Returns full string definition for message of type '<C22C0_PATH>"
  (cl:format cl:nil "float32[] PATH_Left~%float32[] PATH_Right~%float32 PATH_Slope~%int32 PATH_ROU~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C22C0_PATH)))
  "Returns full string definition for message of type 'C22C0_PATH"
  (cl:format cl:nil "float32[] PATH_Left~%float32[] PATH_Right~%float32 PATH_Slope~%int32 PATH_ROU~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C22C0_PATH>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PATH_Left) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PATH_Right) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C22C0_PATH>))
  "Converts a ROS message object to a list"
  (cl:list 'C22C0_PATH
    (cl:cons ':PATH_Left (PATH_Left msg))
    (cl:cons ':PATH_Right (PATH_Right msg))
    (cl:cons ':PATH_Slope (PATH_Slope msg))
    (cl:cons ':PATH_ROU (PATH_ROU msg))
))
