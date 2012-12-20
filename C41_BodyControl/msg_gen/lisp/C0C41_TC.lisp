; Auto-generated. Do not edit!


(cl:in-package C41_BodyControl-msg)


;//! \htmlinclude C0C41_TC.msg.html

(cl:defclass <C0C41_TC> (roslisp-msg-protocol:ros-message)
  ((TC_TORQUE
    :reader TC_TORQUE
    :initarg :TC_TORQUE
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass C0C41_TC (<C0C41_TC>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C41_TC>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C41_TC)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_BodyControl-msg:<C0C41_TC> is deprecated: use C41_BodyControl-msg:C0C41_TC instead.")))

(cl:ensure-generic-function 'TC_TORQUE-val :lambda-list '(m))
(cl:defmethod TC_TORQUE-val ((m <C0C41_TC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:TC_TORQUE-val is deprecated.  Use C41_BodyControl-msg:TC_TORQUE instead.")
  (TC_TORQUE m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C41_TC>) ostream)
  "Serializes a message object of type '<C0C41_TC>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'TC_TORQUE))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'TC_TORQUE))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C41_TC>) istream)
  "Deserializes a message object of type '<C0C41_TC>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'TC_TORQUE) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'TC_TORQUE)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C41_TC>)))
  "Returns string type for a message object of type '<C0C41_TC>"
  "C41_BodyControl/C0C41_TC")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C41_TC)))
  "Returns string type for a message object of type 'C0C41_TC"
  "C41_BodyControl/C0C41_TC")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C41_TC>)))
  "Returns md5sum for a message object of type '<C0C41_TC>"
  "95f314b4f53225c89774ef96cf49d4ae")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C41_TC)))
  "Returns md5sum for a message object of type 'C0C41_TC"
  "95f314b4f53225c89774ef96cf49d4ae")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C41_TC>)))
  "Returns full string definition for message of type '<C0C41_TC>"
  (cl:format cl:nil "float32[] TC_TORQUE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C41_TC)))
  "Returns full string definition for message of type 'C0C41_TC"
  (cl:format cl:nil "float32[] TC_TORQUE~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C41_TC>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'TC_TORQUE) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C41_TC>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C41_TC
    (cl:cons ':TC_TORQUE (TC_TORQUE msg))
))
