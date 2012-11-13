; Auto-generated. Do not edit!


(cl:in-package C41_BodyControl-msg)


;//! \htmlinclude C0C41_PVA.msg.html

(cl:defclass <C0C41_PVA> (roslisp-msg-protocol:ros-message)
  ((PVA_POS
    :reader PVA_POS
    :initarg :PVA_POS
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (PVA_VEL
    :reader PVA_VEL
    :initarg :PVA_VEL
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (PVA_ACC
    :reader PVA_ACC
    :initarg :PVA_ACC
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass C0C41_PVA (<C0C41_PVA>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C41_PVA>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C41_PVA)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_BodyControl-msg:<C0C41_PVA> is deprecated: use C41_BodyControl-msg:C0C41_PVA instead.")))

(cl:ensure-generic-function 'PVA_POS-val :lambda-list '(m))
(cl:defmethod PVA_POS-val ((m <C0C41_PVA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:PVA_POS-val is deprecated.  Use C41_BodyControl-msg:PVA_POS instead.")
  (PVA_POS m))

(cl:ensure-generic-function 'PVA_VEL-val :lambda-list '(m))
(cl:defmethod PVA_VEL-val ((m <C0C41_PVA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:PVA_VEL-val is deprecated.  Use C41_BodyControl-msg:PVA_VEL instead.")
  (PVA_VEL m))

(cl:ensure-generic-function 'PVA_ACC-val :lambda-list '(m))
(cl:defmethod PVA_ACC-val ((m <C0C41_PVA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:PVA_ACC-val is deprecated.  Use C41_BodyControl-msg:PVA_ACC instead.")
  (PVA_ACC m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C41_PVA>) ostream)
  "Serializes a message object of type '<C0C41_PVA>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PVA_POS))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'PVA_POS))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PVA_VEL))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'PVA_VEL))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PVA_ACC))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'PVA_ACC))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C41_PVA>) istream)
  "Deserializes a message object of type '<C0C41_PVA>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'PVA_POS) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PVA_POS)))
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
  (cl:setf (cl:slot-value msg 'PVA_VEL) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PVA_VEL)))
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
  (cl:setf (cl:slot-value msg 'PVA_ACC) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PVA_ACC)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C41_PVA>)))
  "Returns string type for a message object of type '<C0C41_PVA>"
  "C41_BodyControl/C0C41_PVA")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C41_PVA)))
  "Returns string type for a message object of type 'C0C41_PVA"
  "C41_BodyControl/C0C41_PVA")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C41_PVA>)))
  "Returns md5sum for a message object of type '<C0C41_PVA>"
  "bbdb9ca6f014d34732f2356d364199a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C41_PVA)))
  "Returns md5sum for a message object of type 'C0C41_PVA"
  "bbdb9ca6f014d34732f2356d364199a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C41_PVA>)))
  "Returns full string definition for message of type '<C0C41_PVA>"
  (cl:format cl:nil "float32[] PVA_POS~%float32[] PVA_VEL~%float32[] PVA_ACC~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C41_PVA)))
  "Returns full string definition for message of type 'C0C41_PVA"
  (cl:format cl:nil "float32[] PVA_POS~%float32[] PVA_VEL~%float32[] PVA_ACC~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C41_PVA>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PVA_POS) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PVA_VEL) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PVA_ACC) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C41_PVA>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C41_PVA
    (cl:cons ':PVA_POS (PVA_POS msg))
    (cl:cons ':PVA_VEL (PVA_VEL msg))
    (cl:cons ':PVA_ACC (PVA_ACC msg))
))
