; Auto-generated. Do not edit!


(cl:in-package C34_Executer-srv)


;//! \htmlinclude save_file-request.msg.html

(cl:defclass <save_file-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform "")
   (content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass save_file-request (<save_file-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <save_file-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'save_file-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<save_file-request> is deprecated: use C34_Executer-srv:save_file-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <save_file-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:filename-val is deprecated.  Use C34_Executer-srv:filename instead.")
  (filename m))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <save_file-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:content-val is deprecated.  Use C34_Executer-srv:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <save_file-request>) ostream)
  "Serializes a message object of type '<save_file-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <save_file-request>) istream)
  "Deserializes a message object of type '<save_file-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'content) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'content) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<save_file-request>)))
  "Returns string type for a service object of type '<save_file-request>"
  "C34_Executer/save_fileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'save_file-request)))
  "Returns string type for a service object of type 'save_file-request"
  "C34_Executer/save_fileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<save_file-request>)))
  "Returns md5sum for a message object of type '<save_file-request>"
  "d53ed0b067c22e918369e55b6e86c747")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'save_file-request)))
  "Returns md5sum for a message object of type 'save_file-request"
  "d53ed0b067c22e918369e55b6e86c747")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<save_file-request>)))
  "Returns full string definition for message of type '<save_file-request>"
  (cl:format cl:nil "string filename~%string content~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'save_file-request)))
  "Returns full string definition for message of type 'save_file-request"
  (cl:format cl:nil "string filename~%string content~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <save_file-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <save_file-request>))
  "Converts a ROS message object to a list"
  (cl:list 'save_file-request
    (cl:cons ':filename (filename msg))
    (cl:cons ':content (content msg))
))
;//! \htmlinclude save_file-response.msg.html

(cl:defclass <save_file-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass save_file-response (<save_file-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <save_file-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'save_file-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<save_file-response> is deprecated: use C34_Executer-srv:save_file-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <save_file-response>) ostream)
  "Serializes a message object of type '<save_file-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <save_file-response>) istream)
  "Deserializes a message object of type '<save_file-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<save_file-response>)))
  "Returns string type for a service object of type '<save_file-response>"
  "C34_Executer/save_fileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'save_file-response)))
  "Returns string type for a service object of type 'save_file-response"
  "C34_Executer/save_fileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<save_file-response>)))
  "Returns md5sum for a message object of type '<save_file-response>"
  "d53ed0b067c22e918369e55b6e86c747")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'save_file-response)))
  "Returns md5sum for a message object of type 'save_file-response"
  "d53ed0b067c22e918369e55b6e86c747")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<save_file-response>)))
  "Returns full string definition for message of type '<save_file-response>"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'save_file-response)))
  "Returns full string definition for message of type 'save_file-response"
  (cl:format cl:nil "~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <save_file-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <save_file-response>))
  "Converts a ROS message object to a list"
  (cl:list 'save_file-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'save_file)))
  'save_file-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'save_file)))
  'save_file-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'save_file)))
  "Returns string type for a service object of type '<save_file>"
  "C34_Executer/save_file")