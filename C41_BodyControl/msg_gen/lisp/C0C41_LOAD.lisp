; Auto-generated. Do not edit!


(cl:in-package C41_BodyControl-msg)


;//! \htmlinclude C0C41_LOAD.msg.html

(cl:defclass <C0C41_LOAD> (roslisp-msg-protocol:ros-message)
  ((Load
    :reader Load
    :initarg :Load
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass C0C41_LOAD (<C0C41_LOAD>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C41_LOAD>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C41_LOAD)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_BodyControl-msg:<C0C41_LOAD> is deprecated: use C41_BodyControl-msg:C0C41_LOAD instead.")))

(cl:ensure-generic-function 'Load-val :lambda-list '(m))
(cl:defmethod Load-val ((m <C0C41_LOAD>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-msg:Load-val is deprecated.  Use C41_BodyControl-msg:Load instead.")
  (Load m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C41_LOAD>) ostream)
  "Serializes a message object of type '<C0C41_LOAD>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Load))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'Load))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C41_LOAD>) istream)
  "Deserializes a message object of type '<C0C41_LOAD>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Load) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Load)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C41_LOAD>)))
  "Returns string type for a message object of type '<C0C41_LOAD>"
  "C41_BodyControl/C0C41_LOAD")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C41_LOAD)))
  "Returns string type for a message object of type 'C0C41_LOAD"
  "C41_BodyControl/C0C41_LOAD")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C41_LOAD>)))
  "Returns md5sum for a message object of type '<C0C41_LOAD>"
  "15447bde215ee89cf0a0b1e6c67461cc")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C41_LOAD)))
  "Returns md5sum for a message object of type 'C0C41_LOAD"
  "15447bde215ee89cf0a0b1e6c67461cc")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C41_LOAD>)))
  "Returns full string definition for message of type '<C0C41_LOAD>"
  (cl:format cl:nil "float32[] Load~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C41_LOAD)))
  "Returns full string definition for message of type 'C0C41_LOAD"
  (cl:format cl:nil "float32[] Load~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C41_LOAD>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Load) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C41_LOAD>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C41_LOAD
    (cl:cons ':Load (Load msg))
))
