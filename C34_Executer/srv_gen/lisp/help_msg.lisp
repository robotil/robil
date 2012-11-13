; Auto-generated. Do not edit!


(cl:in-package C34_Executer-srv)


;//! \htmlinclude help_msg-request.msg.html

(cl:defclass <help_msg-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass help_msg-request (<help_msg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <help_msg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'help_msg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<help_msg-request> is deprecated: use C34_Executer-srv:help_msg-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <help_msg-request>) ostream)
  "Serializes a message object of type '<help_msg-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <help_msg-request>) istream)
  "Deserializes a message object of type '<help_msg-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<help_msg-request>)))
  "Returns string type for a service object of type '<help_msg-request>"
  "C34_Executer/help_msgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'help_msg-request)))
  "Returns string type for a service object of type 'help_msg-request"
  "C34_Executer/help_msgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<help_msg-request>)))
  "Returns md5sum for a message object of type '<help_msg-request>"
  "74697ed3d931f6eede8bf3a8dfeca160")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'help_msg-request)))
  "Returns md5sum for a message object of type 'help_msg-request"
  "74697ed3d931f6eede8bf3a8dfeca160")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<help_msg-request>)))
  "Returns full string definition for message of type '<help_msg-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'help_msg-request)))
  "Returns full string definition for message of type 'help_msg-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <help_msg-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <help_msg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'help_msg-request
))
;//! \htmlinclude help_msg-response.msg.html

(cl:defclass <help_msg-response> (roslisp-msg-protocol:ros-message)
  ((text
    :reader text
    :initarg :text
    :type cl:string
    :initform ""))
)

(cl:defclass help_msg-response (<help_msg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <help_msg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'help_msg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<help_msg-response> is deprecated: use C34_Executer-srv:help_msg-response instead.")))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <help_msg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:text-val is deprecated.  Use C34_Executer-srv:text instead.")
  (text m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <help_msg-response>) ostream)
  "Serializes a message object of type '<help_msg-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <help_msg-response>) istream)
  "Deserializes a message object of type '<help_msg-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<help_msg-response>)))
  "Returns string type for a service object of type '<help_msg-response>"
  "C34_Executer/help_msgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'help_msg-response)))
  "Returns string type for a service object of type 'help_msg-response"
  "C34_Executer/help_msgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<help_msg-response>)))
  "Returns md5sum for a message object of type '<help_msg-response>"
  "74697ed3d931f6eede8bf3a8dfeca160")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'help_msg-response)))
  "Returns md5sum for a message object of type 'help_msg-response"
  "74697ed3d931f6eede8bf3a8dfeca160")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<help_msg-response>)))
  "Returns full string definition for message of type '<help_msg-response>"
  (cl:format cl:nil "string text~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'help_msg-response)))
  "Returns full string definition for message of type 'help_msg-response"
  (cl:format cl:nil "string text~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <help_msg-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'text))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <help_msg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'help_msg-response
    (cl:cons ':text (text msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'help_msg)))
  'help_msg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'help_msg)))
  'help_msg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'help_msg)))
  "Returns string type for a service object of type '<help_msg>"
  "C34_Executer/help_msg")