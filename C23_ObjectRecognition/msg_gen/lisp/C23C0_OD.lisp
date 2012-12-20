; Auto-generated. Do not edit!


(cl:in-package C23_ObjectRecognition-msg)


;//! \htmlinclude C23C0_OD.msg.html

(cl:defclass <C23C0_OD> (roslisp-msg-protocol:ros-message)
  ((ObjectDetected
    :reader ObjectDetected
    :initarg :ObjectDetected
    :type cl:integer
    :initform 0))
)

(cl:defclass C23C0_OD (<C23C0_OD>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C23C0_OD>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C23C0_OD)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-msg:<C23C0_OD> is deprecated: use C23_ObjectRecognition-msg:C23C0_OD instead.")))

(cl:ensure-generic-function 'ObjectDetected-val :lambda-list '(m))
(cl:defmethod ObjectDetected-val ((m <C23C0_OD>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:ObjectDetected-val is deprecated.  Use C23_ObjectRecognition-msg:ObjectDetected instead.")
  (ObjectDetected m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C23C0_OD>) ostream)
  "Serializes a message object of type '<C23C0_OD>"
  (cl:let* ((signed (cl:slot-value msg 'ObjectDetected)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C23C0_OD>) istream)
  "Deserializes a message object of type '<C23C0_OD>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'ObjectDetected) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C23C0_OD>)))
  "Returns string type for a message object of type '<C23C0_OD>"
  "C23_ObjectRecognition/C23C0_OD")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23C0_OD)))
  "Returns string type for a message object of type 'C23C0_OD"
  "C23_ObjectRecognition/C23C0_OD")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C23C0_OD>)))
  "Returns md5sum for a message object of type '<C23C0_OD>"
  "8b08a7adabc5e7d0af82a11bf4a6523e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C23C0_OD)))
  "Returns md5sum for a message object of type 'C23C0_OD"
  "8b08a7adabc5e7d0af82a11bf4a6523e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C23C0_OD>)))
  "Returns full string definition for message of type '<C23C0_OD>"
  (cl:format cl:nil "int32 ObjectDetected~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C23C0_OD)))
  "Returns full string definition for message of type 'C23C0_OD"
  (cl:format cl:nil "int32 ObjectDetected~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C23C0_OD>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C23C0_OD>))
  "Converts a ROS message object to a list"
  (cl:list 'C23C0_OD
    (cl:cons ':ObjectDetected (ObjectDetected msg))
))
