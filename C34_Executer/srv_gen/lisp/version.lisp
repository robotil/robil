; Auto-generated. Do not edit!


(cl:in-package C34_Executer-srv)


;//! \htmlinclude version-request.msg.html

(cl:defclass <version-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass version-request (<version-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <version-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'version-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<version-request> is deprecated: use C34_Executer-srv:version-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <version-request>) ostream)
  "Serializes a message object of type '<version-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <version-request>) istream)
  "Deserializes a message object of type '<version-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<version-request>)))
  "Returns string type for a service object of type '<version-request>"
  "C34_Executer/versionRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'version-request)))
  "Returns string type for a service object of type 'version-request"
  "C34_Executer/versionRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<version-request>)))
  "Returns md5sum for a message object of type '<version-request>"
  "1a6f3c569a7ebe01c997148cb8f0f453")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'version-request)))
  "Returns md5sum for a message object of type 'version-request"
  "1a6f3c569a7ebe01c997148cb8f0f453")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<version-request>)))
  "Returns full string definition for message of type '<version-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'version-request)))
  "Returns full string definition for message of type 'version-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <version-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <version-request>))
  "Converts a ROS message object to a list"
  (cl:list 'version-request
))
;//! \htmlinclude version-response.msg.html

(cl:defclass <version-response> (roslisp-msg-protocol:ros-message)
  ((version
    :reader version
    :initarg :version
    :type cl:string
    :initform ""))
)

(cl:defclass version-response (<version-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <version-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'version-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<version-response> is deprecated: use C34_Executer-srv:version-response instead.")))

(cl:ensure-generic-function 'version-val :lambda-list '(m))
(cl:defmethod version-val ((m <version-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:version-val is deprecated.  Use C34_Executer-srv:version instead.")
  (version m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <version-response>) ostream)
  "Serializes a message object of type '<version-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'version))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'version))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <version-response>) istream)
  "Deserializes a message object of type '<version-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'version) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'version) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<version-response>)))
  "Returns string type for a service object of type '<version-response>"
  "C34_Executer/versionResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'version-response)))
  "Returns string type for a service object of type 'version-response"
  "C34_Executer/versionResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<version-response>)))
  "Returns md5sum for a message object of type '<version-response>"
  "1a6f3c569a7ebe01c997148cb8f0f453")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'version-response)))
  "Returns md5sum for a message object of type 'version-response"
  "1a6f3c569a7ebe01c997148cb8f0f453")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<version-response>)))
  "Returns full string definition for message of type '<version-response>"
  (cl:format cl:nil "string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'version-response)))
  "Returns full string definition for message of type 'version-response"
  (cl:format cl:nil "string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <version-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'version))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <version-response>))
  "Converts a ROS message object to a list"
  (cl:list 'version-response
    (cl:cons ':version (version msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'version)))
  'version-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'version)))
  'version-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'version)))
  "Returns string type for a service object of type '<version>"
  "C34_Executer/version")