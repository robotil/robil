; Auto-generated. Do not edit!


(cl:in-package Executer-srv)


;//! \htmlinclude btstack-request.msg.html

(cl:defclass <btstack-request> (roslisp-msg-protocol:ros-message)
  ((tree_id
    :reader tree_id
    :initarg :tree_id
    :type cl:string
    :initform ""))
)

(cl:defclass btstack-request (<btstack-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <btstack-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'btstack-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Executer-srv:<btstack-request> is deprecated: use Executer-srv:btstack-request instead.")))

(cl:ensure-generic-function 'tree_id-val :lambda-list '(m))
(cl:defmethod tree_id-val ((m <btstack-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Executer-srv:tree_id-val is deprecated.  Use Executer-srv:tree_id instead.")
  (tree_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <btstack-request>) ostream)
  "Serializes a message object of type '<btstack-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tree_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tree_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <btstack-request>) istream)
  "Deserializes a message object of type '<btstack-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tree_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tree_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<btstack-request>)))
  "Returns string type for a service object of type '<btstack-request>"
  "Executer/btstackRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'btstack-request)))
  "Returns string type for a service object of type 'btstack-request"
  "Executer/btstackRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<btstack-request>)))
  "Returns md5sum for a message object of type '<btstack-request>"
  "4787e2d1c6f5996e8efbfd4e07122e4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'btstack-request)))
  "Returns md5sum for a message object of type 'btstack-request"
  "4787e2d1c6f5996e8efbfd4e07122e4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<btstack-request>)))
  "Returns full string definition for message of type '<btstack-request>"
  (cl:format cl:nil "string tree_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'btstack-request)))
  "Returns full string definition for message of type 'btstack-request"
  (cl:format cl:nil "string tree_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <btstack-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tree_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <btstack-request>))
  "Converts a ROS message object to a list"
  (cl:list 'btstack-request
    (cl:cons ':tree_id (tree_id msg))
))
;//! \htmlinclude btstack-response.msg.html

(cl:defclass <btstack-response> (roslisp-msg-protocol:ros-message)
  ((content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass btstack-response (<btstack-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <btstack-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'btstack-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Executer-srv:<btstack-response> is deprecated: use Executer-srv:btstack-response instead.")))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <btstack-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Executer-srv:content-val is deprecated.  Use Executer-srv:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <btstack-response>) ostream)
  "Serializes a message object of type '<btstack-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <btstack-response>) istream)
  "Deserializes a message object of type '<btstack-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<btstack-response>)))
  "Returns string type for a service object of type '<btstack-response>"
  "Executer/btstackResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'btstack-response)))
  "Returns string type for a service object of type 'btstack-response"
  "Executer/btstackResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<btstack-response>)))
  "Returns md5sum for a message object of type '<btstack-response>"
  "4787e2d1c6f5996e8efbfd4e07122e4d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'btstack-response)))
  "Returns md5sum for a message object of type 'btstack-response"
  "4787e2d1c6f5996e8efbfd4e07122e4d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<btstack-response>)))
  "Returns full string definition for message of type '<btstack-response>"
  (cl:format cl:nil "string content~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'btstack-response)))
  "Returns full string definition for message of type 'btstack-response"
  (cl:format cl:nil "string content~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <btstack-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <btstack-response>))
  "Converts a ROS message object to a list"
  (cl:list 'btstack-response
    (cl:cons ':content (content msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'btstack)))
  'btstack-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'btstack)))
  'btstack-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'btstack)))
  "Returns string type for a service object of type '<btstack>"
  "Executer/btstack")