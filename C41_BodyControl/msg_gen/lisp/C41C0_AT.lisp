; Auto-generated. Do not edit!


(cl:in-package C41_BodyControl-msg)


;//! \htmlinclude C41C0_AT.msg.html

(cl:defclass <C41C0_AT> (roslisp-msg-protocol:ros-message)
  ((Actual_torque
    :reader Actual_torque
    :initarg :Actual_torque
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass C41C0_AT (<C41C0_AT>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C41C0_AT>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C41C0_AT)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_BodyControl-msg:<C41C0_AT> is deprecated: use C41_BodyControl-msg:C41C0_AT instead.")))

(cl:ensure-generic-function 'Actual_torque-val :lambda-list '(m))
(cl:defmethod Actual_torque-val ((m <C41C0_AT>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:Actual_torque-val is deprecated.  Use C41_BodyControl-msg:Actual_torque instead.")
  (Actual_torque m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C41C0_AT>) ostream)
  "Serializes a message object of type '<C41C0_AT>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Actual_torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Actual_torque))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C41C0_AT>) istream)
  "Deserializes a message object of type '<C41C0_AT>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Actual_torque) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Actual_torque)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C41C0_AT>)))
  "Returns string type for a message object of type '<C41C0_AT>"
  "C41_BodyControl/C41C0_AT")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C41C0_AT)))
  "Returns string type for a message object of type 'C41C0_AT"
  "C41_BodyControl/C41C0_AT")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C41C0_AT>)))
  "Returns md5sum for a message object of type '<C41C0_AT>"
  "a5c3064b8b8d3738379e0d1cc54f0366")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C41C0_AT)))
  "Returns md5sum for a message object of type 'C41C0_AT"
  "a5c3064b8b8d3738379e0d1cc54f0366")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C41C0_AT>)))
  "Returns full string definition for message of type '<C41C0_AT>"
  (cl:format cl:nil "float32[] Actual_torque ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C41C0_AT)))
  "Returns full string definition for message of type 'C41C0_AT"
  (cl:format cl:nil "float32[] Actual_torque ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C41C0_AT>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Actual_torque) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C41C0_AT>))
  "Converts a ROS message object to a list"
  (cl:list 'C41C0_AT
    (cl:cons ':Actual_torque (Actual_torque msg))
))
