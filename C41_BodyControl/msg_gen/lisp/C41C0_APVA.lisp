; Auto-generated. Do not edit!


(cl:in-package C41_BodyControl-msg)


;//! \htmlinclude C41C0_APVA.msg.html

(cl:defclass <C41C0_APVA> (roslisp-msg-protocol:ros-message)
  ((Actual_Position
    :reader Actual_Position
    :initarg :Actual_Position
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (Actual_Velocity
    :reader Actual_Velocity
    :initarg :Actual_Velocity
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (Actual_Acceleration
    :reader Actual_Acceleration
    :initarg :Actual_Acceleration
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass C41C0_APVA (<C41C0_APVA>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C41C0_APVA>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C41C0_APVA)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_BodyControl-msg:<C41C0_APVA> is deprecated: use C41_BodyControl-msg:C41C0_APVA instead.")))

(cl:ensure-generic-function 'Actual_Position-val :lambda-list '(m))
(cl:defmethod Actual_Position-val ((m <C41C0_APVA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:Actual_Position-val is deprecated.  Use C41_BodyControl-msg:Actual_Position instead.")
  (Actual_Position m))

(cl:ensure-generic-function 'Actual_Velocity-val :lambda-list '(m))
(cl:defmethod Actual_Velocity-val ((m <C41C0_APVA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:Actual_Velocity-val is deprecated.  Use C41_BodyControl-msg:Actual_Velocity instead.")
  (Actual_Velocity m))

(cl:ensure-generic-function 'Actual_Acceleration-val :lambda-list '(m))
(cl:defmethod Actual_Acceleration-val ((m <C41C0_APVA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:Actual_Acceleration-val is deprecated.  Use C41_BodyControl-msg:Actual_Acceleration instead.")
  (Actual_Acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C41C0_APVA>) ostream)
  "Serializes a message object of type '<C41C0_APVA>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Actual_Position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Actual_Position))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Actual_Velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Actual_Velocity))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Actual_Acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Actual_Acceleration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C41C0_APVA>) istream)
  "Deserializes a message object of type '<C41C0_APVA>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Actual_Position) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Actual_Position)))
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
  (cl:setf (cl:slot-value msg 'Actual_Velocity) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Actual_Velocity)))
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
  (cl:setf (cl:slot-value msg 'Actual_Acceleration) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Actual_Acceleration)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C41C0_APVA>)))
  "Returns string type for a message object of type '<C41C0_APVA>"
  "C41_BodyControl/C41C0_APVA")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C41C0_APVA)))
  "Returns string type for a message object of type 'C41C0_APVA"
  "C41_BodyControl/C41C0_APVA")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C41C0_APVA>)))
  "Returns md5sum for a message object of type '<C41C0_APVA>"
  "86ecff566b892a98eeaf18b12292a99c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C41C0_APVA)))
  "Returns md5sum for a message object of type 'C41C0_APVA"
  "86ecff566b892a98eeaf18b12292a99c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C41C0_APVA>)))
  "Returns full string definition for message of type '<C41C0_APVA>"
  (cl:format cl:nil "float32[] Actual_Position~%float32[] Actual_Velocity~%float32[] Actual_Acceleration ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C41C0_APVA)))
  "Returns full string definition for message of type 'C41C0_APVA"
  (cl:format cl:nil "float32[] Actual_Position~%float32[] Actual_Velocity~%float32[] Actual_Acceleration ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C41C0_APVA>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Actual_Position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Actual_Velocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Actual_Acceleration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C41C0_APVA>))
  "Converts a ROS message object to a list"
  (cl:list 'C41C0_APVA
    (cl:cons ':Actual_Position (Actual_Position msg))
    (cl:cons ':Actual_Velocity (Actual_Velocity msg))
    (cl:cons ':Actual_Acceleration (Actual_Acceleration msg))
))
