; Auto-generated. Do not edit!


(cl:in-package C43_LocalBodyPVA-msg)


;//! \htmlinclude C0C43_SL.msg.html

(cl:defclass <C0C43_SL> (roslisp-msg-protocol:ros-message)
  ((Selected_Link
    :reader Selected_Link
    :initarg :Selected_Link
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass C0C43_SL (<C0C43_SL>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C43_SL>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C43_SL)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C43_LocalBodyPVA-msg:<C0C43_SL> is deprecated: use C43_LocalBodyPVA-msg:C0C43_SL instead.")))

(cl:ensure-generic-function 'Selected_Link-val :lambda-list '(m))
(cl:defmethod Selected_Link-val ((m <C0C43_SL>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:Selected_Link-val is deprecated.  Use C43_LocalBodyPVA-msg:Selected_Link instead.")
  (Selected_Link m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C43_SL>) ostream)
  "Serializes a message object of type '<C0C43_SL>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Selected_Link))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'Selected_Link))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C43_SL>) istream)
  "Deserializes a message object of type '<C0C43_SL>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Selected_Link) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Selected_Link)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C43_SL>)))
  "Returns string type for a message object of type '<C0C43_SL>"
  "C43_LocalBodyPVA/C0C43_SL")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C43_SL)))
  "Returns string type for a message object of type 'C0C43_SL"
  "C43_LocalBodyPVA/C0C43_SL")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C43_SL>)))
  "Returns md5sum for a message object of type '<C0C43_SL>"
  "61fd1383bd51ca5f0bdb7f73daa2ad8e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C43_SL)))
  "Returns md5sum for a message object of type 'C0C43_SL"
  "61fd1383bd51ca5f0bdb7f73daa2ad8e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C43_SL>)))
  "Returns full string definition for message of type '<C0C43_SL>"
  (cl:format cl:nil "int32[] Selected_Link~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C43_SL)))
  "Returns full string definition for message of type 'C0C43_SL"
  (cl:format cl:nil "int32[] Selected_Link~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C43_SL>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Selected_Link) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C43_SL>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C43_SL
    (cl:cons ':Selected_Link (Selected_Link msg))
))
