; Auto-generated. Do not edit!


(cl:in-package C44_ClimbLadder-msg)


;//! \htmlinclude C23C44_LDIM.msg.html

(cl:defclass <C23C44_LDIM> (roslisp-msg-protocol:ros-message)
  ((ladder_dimensions
    :reader ladder_dimensions
    :initarg :ladder_dimensions
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass C23C44_LDIM (<C23C44_LDIM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C23C44_LDIM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C23C44_LDIM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C44_ClimbLadder-msg:<C23C44_LDIM> is deprecated: use C44_ClimbLadder-msg:C23C44_LDIM instead.")))

(cl:ensure-generic-function 'ladder_dimensions-val :lambda-list '(m))
(cl:defmethod ladder_dimensions-val ((m <C23C44_LDIM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C44_ClimbLadder-msg:ladder_dimensions-val is deprecated.  Use C44_ClimbLadder-msg:ladder_dimensions instead.")
  (ladder_dimensions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C23C44_LDIM>) ostream)
  "Serializes a message object of type '<C23C44_LDIM>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ladder_dimensions))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'ladder_dimensions))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C23C44_LDIM>) istream)
  "Deserializes a message object of type '<C23C44_LDIM>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ladder_dimensions) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ladder_dimensions)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C23C44_LDIM>)))
  "Returns string type for a message object of type '<C23C44_LDIM>"
  "C44_ClimbLadder/C23C44_LDIM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23C44_LDIM)))
  "Returns string type for a message object of type 'C23C44_LDIM"
  "C44_ClimbLadder/C23C44_LDIM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C23C44_LDIM>)))
  "Returns md5sum for a message object of type '<C23C44_LDIM>"
  "ba8c4f2a2a20475cf4d20206fa52bd50")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C23C44_LDIM)))
  "Returns md5sum for a message object of type 'C23C44_LDIM"
  "ba8c4f2a2a20475cf4d20206fa52bd50")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C23C44_LDIM>)))
  "Returns full string definition for message of type '<C23C44_LDIM>"
  (cl:format cl:nil "float32[] ladder_dimensions~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C23C44_LDIM)))
  "Returns full string definition for message of type 'C23C44_LDIM"
  (cl:format cl:nil "float32[] ladder_dimensions~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C23C44_LDIM>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ladder_dimensions) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C23C44_LDIM>))
  "Converts a ROS message object to a list"
  (cl:list 'C23C44_LDIM
    (cl:cons ':ladder_dimensions (ladder_dimensions msg))
))
