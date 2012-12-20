; Auto-generated. Do not edit!


(cl:in-package C34_Executer-srv)


;//! \htmlinclude read_file-request.msg.html

(cl:defclass <read_file-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass read_file-request (<read_file-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <read_file-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'read_file-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<read_file-request> is deprecated: use C34_Executer-srv:read_file-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <read_file-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:filename-val is deprecated.  Use C34_Executer-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <read_file-request>) ostream)
  "Serializes a message object of type '<read_file-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <read_file-request>) istream)
  "Deserializes a message object of type '<read_file-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'filename) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'filename) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<read_file-request>)))
  "Returns string type for a service object of type '<read_file-request>"
  "C34_Executer/read_fileRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'read_file-request)))
  "Returns string type for a service object of type 'read_file-request"
  "C34_Executer/read_fileRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<read_file-request>)))
  "Returns md5sum for a message object of type '<read_file-request>"
  "721a81a841b9f7d23627ace8018b523f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'read_file-request)))
  "Returns md5sum for a message object of type 'read_file-request"
  "721a81a841b9f7d23627ace8018b523f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<read_file-request>)))
  "Returns full string definition for message of type '<read_file-request>"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'read_file-request)))
  "Returns full string definition for message of type 'read_file-request"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <read_file-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <read_file-request>))
  "Converts a ROS message object to a list"
  (cl:list 'read_file-request
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude read_file-response.msg.html

(cl:defclass <read_file-response> (roslisp-msg-protocol:ros-message)
  ((content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass read_file-response (<read_file-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <read_file-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'read_file-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<read_file-response> is deprecated: use C34_Executer-srv:read_file-response instead.")))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <read_file-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:content-val is deprecated.  Use C34_Executer-srv:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <read_file-response>) ostream)
  "Serializes a message object of type '<read_file-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <read_file-response>) istream)
  "Deserializes a message object of type '<read_file-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<read_file-response>)))
  "Returns string type for a service object of type '<read_file-response>"
  "C34_Executer/read_fileResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'read_file-response)))
  "Returns string type for a service object of type 'read_file-response"
  "C34_Executer/read_fileResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<read_file-response>)))
  "Returns md5sum for a message object of type '<read_file-response>"
  "721a81a841b9f7d23627ace8018b523f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'read_file-response)))
  "Returns md5sum for a message object of type 'read_file-response"
  "721a81a841b9f7d23627ace8018b523f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<read_file-response>)))
  "Returns full string definition for message of type '<read_file-response>"
  (cl:format cl:nil "string content~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'read_file-response)))
  "Returns full string definition for message of type 'read_file-response"
  (cl:format cl:nil "string content~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <read_file-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <read_file-response>))
  "Converts a ROS message object to a list"
  (cl:list 'read_file-response
    (cl:cons ':content (content msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'read_file)))
  'read_file-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'read_file)))
  'read_file-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'read_file)))
  "Returns string type for a service object of type '<read_file>"
  "C34_Executer/read_file")