; Auto-generated. Do not edit!


(cl:in-package C11_Agent-msg)


;//! \htmlinclude C11C32_PATH.msg.html

(cl:defclass <C11C32_PATH> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector C11_Agent-msg:pathLocation)
   :initform (cl:make-array 0 :element-type 'C11_Agent-msg:pathLocation :initial-element (cl:make-instance 'C11_Agent-msg:pathLocation))))
)

(cl:defclass C11C32_PATH (<C11C32_PATH>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11C32_PATH>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11C32_PATH)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-msg:<C11C32_PATH> is deprecated: use C11_Agent-msg:C11C32_PATH instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <C11C32_PATH>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-msg:points-val is deprecated.  Use C11_Agent-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11C32_PATH>) ostream)
  "Serializes a message object of type '<C11C32_PATH>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11C32_PATH>) istream)
  "Deserializes a message object of type '<C11C32_PATH>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'C11_Agent-msg:pathLocation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11C32_PATH>)))
  "Returns string type for a message object of type '<C11C32_PATH>"
  "C11_Agent/C11C32_PATH")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11C32_PATH)))
  "Returns string type for a message object of type 'C11C32_PATH"
  "C11_Agent/C11C32_PATH")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11C32_PATH>)))
  "Returns md5sum for a message object of type '<C11C32_PATH>"
  "ffd74dfa964a04e3f477faa3684da1c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11C32_PATH)))
  "Returns md5sum for a message object of type 'C11C32_PATH"
  "ffd74dfa964a04e3f477faa3684da1c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11C32_PATH>)))
  "Returns full string definition for message of type '<C11C32_PATH>"
  (cl:format cl:nil "C11_Agent/pathLocation[] points~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11C32_PATH)))
  "Returns full string definition for message of type 'C11C32_PATH"
  (cl:format cl:nil "C11_Agent/pathLocation[] points~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11C32_PATH>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11C32_PATH>))
  "Converts a ROS message object to a list"
  (cl:list 'C11C32_PATH
    (cl:cons ':points (points msg))
))
