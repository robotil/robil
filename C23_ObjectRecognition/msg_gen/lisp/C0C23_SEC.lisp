; Auto-generated. Do not edit!


(cl:in-package C23_ObjectRecognition-msg)


;//! \htmlinclude C0C23_SEC.msg.html

(cl:defclass <C0C23_SEC> (roslisp-msg-protocol:ros-message)
  ((searchMode
    :reader searchMode
    :initarg :searchMode
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C23_SEC (<C0C23_SEC>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C23_SEC>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C23_SEC)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-msg:<C0C23_SEC> is deprecated: use C23_ObjectRecognition-msg:C0C23_SEC instead.")))

(cl:ensure-generic-function 'searchMode-val :lambda-list '(m))
(cl:defmethod searchMode-val ((m <C0C23_SEC>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:searchMode-val is deprecated.  Use C23_ObjectRecognition-msg:searchMode instead.")
  (searchMode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C23_SEC>) ostream)
  "Serializes a message object of type '<C0C23_SEC>"
  (cl:let* ((signed (cl:slot-value msg 'searchMode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C23_SEC>) istream)
  "Deserializes a message object of type '<C0C23_SEC>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'searchMode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C23_SEC>)))
  "Returns string type for a message object of type '<C0C23_SEC>"
  "C23_ObjectRecognition/C0C23_SEC")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C23_SEC)))
  "Returns string type for a message object of type 'C0C23_SEC"
  "C23_ObjectRecognition/C0C23_SEC")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C23_SEC>)))
  "Returns md5sum for a message object of type '<C0C23_SEC>"
  "bae3fff7963613e512fc7eb6fb3103b8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C23_SEC)))
  "Returns md5sum for a message object of type 'C0C23_SEC"
  "bae3fff7963613e512fc7eb6fb3103b8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C23_SEC>)))
  "Returns full string definition for message of type '<C0C23_SEC>"
  (cl:format cl:nil "int32 searchMode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C23_SEC)))
  "Returns full string definition for message of type 'C0C23_SEC"
  (cl:format cl:nil "int32 searchMode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C23_SEC>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C23_SEC>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C23_SEC
    (cl:cons ':searchMode (searchMode msg))
))
