; Auto-generated. Do not edit!


(cl:in-package C43_LocalBodyPVA-msg)


;//! \htmlinclude C43C0_JPVA.msg.html

(cl:defclass <C43C0_JPVA> (roslisp-msg-protocol:ros-message)
  ((PVA_of_requested_joint
    :reader PVA_of_requested_joint
    :initarg :PVA_of_requested_joint
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass C43C0_JPVA (<C43C0_JPVA>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C43C0_JPVA>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C43C0_JPVA)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C43_LocalBodyPVA-msg:<C43C0_JPVA> is deprecated: use C43_LocalBodyPVA-msg:C43C0_JPVA instead.")))

(cl:ensure-generic-function 'PVA_of_requested_joint-val :lambda-list '(m))
(cl:defmethod PVA_of_requested_joint-val ((m <C43C0_JPVA>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:PVA_of_requested_joint-val is deprecated.  Use C43_LocalBodyPVA-msg:PVA_of_requested_joint instead.")
  (PVA_of_requested_joint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C43C0_JPVA>) ostream)
  "Serializes a message object of type '<C43C0_JPVA>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'PVA_of_requested_joint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'PVA_of_requested_joint))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C43C0_JPVA>) istream)
  "Deserializes a message object of type '<C43C0_JPVA>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'PVA_of_requested_joint) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'PVA_of_requested_joint)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C43C0_JPVA>)))
  "Returns string type for a message object of type '<C43C0_JPVA>"
  "C43_LocalBodyPVA/C43C0_JPVA")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C43C0_JPVA)))
  "Returns string type for a message object of type 'C43C0_JPVA"
  "C43_LocalBodyPVA/C43C0_JPVA")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C43C0_JPVA>)))
  "Returns md5sum for a message object of type '<C43C0_JPVA>"
  "02b806a661d4b34af7a4c1ce3f528754")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C43C0_JPVA)))
  "Returns md5sum for a message object of type 'C43C0_JPVA"
  "02b806a661d4b34af7a4c1ce3f528754")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C43C0_JPVA>)))
  "Returns full string definition for message of type '<C43C0_JPVA>"
  (cl:format cl:nil "float32[] PVA_of_requested_joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C43C0_JPVA)))
  "Returns full string definition for message of type 'C43C0_JPVA"
  (cl:format cl:nil "float32[] PVA_of_requested_joint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C43C0_JPVA>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'PVA_of_requested_joint) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C43C0_JPVA>))
  "Converts a ROS message object to a list"
  (cl:list 'C43C0_JPVA
    (cl:cons ':PVA_of_requested_joint (PVA_of_requested_joint msg))
))
