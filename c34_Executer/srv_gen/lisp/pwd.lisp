; Auto-generated. Do not edit!


(cl:in-package c34_Executer-srv)


;//! \htmlinclude pwd-request.msg.html

(cl:defclass <pwd-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass pwd-request (<pwd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pwd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pwd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name c34_Executer-srv:<pwd-request> is deprecated: use c34_Executer-srv:pwd-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pwd-request>) ostream)
  "Serializes a message object of type '<pwd-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pwd-request>) istream)
  "Deserializes a message object of type '<pwd-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pwd-request>)))
  "Returns string type for a service object of type '<pwd-request>"
  "c34_Executer/pwdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pwd-request)))
  "Returns string type for a service object of type 'pwd-request"
  "c34_Executer/pwdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pwd-request>)))
  "Returns md5sum for a message object of type '<pwd-request>"
  "03da474bc61cfeb81a8854b4ca05bafa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pwd-request)))
  "Returns md5sum for a message object of type 'pwd-request"
  "03da474bc61cfeb81a8854b4ca05bafa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pwd-request>)))
  "Returns full string definition for message of type '<pwd-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pwd-request)))
  "Returns full string definition for message of type 'pwd-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pwd-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pwd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'pwd-request
))
;//! \htmlinclude pwd-response.msg.html

(cl:defclass <pwd-response> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type cl:string
    :initform ""))
)

(cl:defclass pwd-response (<pwd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pwd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pwd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name c34_Executer-srv:<pwd-response> is deprecated: use c34_Executer-srv:pwd-response instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <pwd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader c34_Executer-srv:location-val is deprecated.  Use c34_Executer-srv:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pwd-response>) ostream)
  "Serializes a message object of type '<pwd-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'location))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pwd-response>) istream)
  "Deserializes a message object of type '<pwd-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'location) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pwd-response>)))
  "Returns string type for a service object of type '<pwd-response>"
  "c34_Executer/pwdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pwd-response)))
  "Returns string type for a service object of type 'pwd-response"
  "c34_Executer/pwdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pwd-response>)))
  "Returns md5sum for a message object of type '<pwd-response>"
  "03da474bc61cfeb81a8854b4ca05bafa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pwd-response)))
  "Returns md5sum for a message object of type 'pwd-response"
  "03da474bc61cfeb81a8854b4ca05bafa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pwd-response>)))
  "Returns full string definition for message of type '<pwd-response>"
  (cl:format cl:nil "string location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pwd-response)))
  "Returns full string definition for message of type 'pwd-response"
  (cl:format cl:nil "string location~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pwd-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'location))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pwd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'pwd-response
    (cl:cons ':location (location msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'pwd)))
  'pwd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'pwd)))
  'pwd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pwd)))
  "Returns string type for a service object of type '<pwd>"
  "c34_Executer/pwd")