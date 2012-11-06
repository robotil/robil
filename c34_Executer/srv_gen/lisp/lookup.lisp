; Auto-generated. Do not edit!


(cl:in-package Executer-srv)


;//! \htmlinclude lookup-request.msg.html

(cl:defclass <lookup-request> (roslisp-msg-protocol:ros-message)
  ((filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass lookup-request (<lookup-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lookup-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lookup-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Executer-srv:<lookup-request> is deprecated: use Executer-srv:lookup-request instead.")))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <lookup-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Executer-srv:filename-val is deprecated.  Use Executer-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lookup-request>) ostream)
  "Serializes a message object of type '<lookup-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lookup-request>) istream)
  "Deserializes a message object of type '<lookup-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lookup-request>)))
  "Returns string type for a service object of type '<lookup-request>"
  "Executer/lookupRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lookup-request)))
  "Returns string type for a service object of type 'lookup-request"
  "Executer/lookupRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lookup-request>)))
  "Returns md5sum for a message object of type '<lookup-request>"
  "f3cba7aefdafd396c5a52623838430b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lookup-request)))
  "Returns md5sum for a message object of type 'lookup-request"
  "f3cba7aefdafd396c5a52623838430b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lookup-request>)))
  "Returns full string definition for message of type '<lookup-request>"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lookup-request)))
  "Returns full string definition for message of type 'lookup-request"
  (cl:format cl:nil "string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lookup-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lookup-request>))
  "Converts a ROS message object to a list"
  (cl:list 'lookup-request
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude lookup-response.msg.html

(cl:defclass <lookup-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass lookup-response (<lookup-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <lookup-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'lookup-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Executer-srv:<lookup-response> is deprecated: use Executer-srv:lookup-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <lookup-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Executer-srv:output-val is deprecated.  Use Executer-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <lookup-response>) ostream)
  "Serializes a message object of type '<lookup-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <lookup-response>) istream)
  "Deserializes a message object of type '<lookup-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'output) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'output) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<lookup-response>)))
  "Returns string type for a service object of type '<lookup-response>"
  "Executer/lookupResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lookup-response)))
  "Returns string type for a service object of type 'lookup-response"
  "Executer/lookupResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<lookup-response>)))
  "Returns md5sum for a message object of type '<lookup-response>"
  "f3cba7aefdafd396c5a52623838430b7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'lookup-response)))
  "Returns md5sum for a message object of type 'lookup-response"
  "f3cba7aefdafd396c5a52623838430b7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<lookup-response>)))
  "Returns full string definition for message of type '<lookup-response>"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'lookup-response)))
  "Returns full string definition for message of type 'lookup-response"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <lookup-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <lookup-response>))
  "Converts a ROS message object to a list"
  (cl:list 'lookup-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'lookup)))
  'lookup-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'lookup)))
  'lookup-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'lookup)))
  "Returns string type for a service object of type '<lookup>"
  "Executer/lookup")