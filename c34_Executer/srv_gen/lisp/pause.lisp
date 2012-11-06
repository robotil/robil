; Auto-generated. Do not edit!


(cl:in-package Executer-srv)


;//! \htmlinclude pause-request.msg.html

(cl:defclass <pause-request> (roslisp-msg-protocol:ros-message)
  ((tree_id
    :reader tree_id
    :initarg :tree_id
    :type cl:string
    :initform ""))
)

(cl:defclass pause-request (<pause-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pause-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pause-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Executer-srv:<pause-request> is deprecated: use Executer-srv:pause-request instead.")))

(cl:ensure-generic-function 'tree_id-val :lambda-list '(m))
(cl:defmethod tree_id-val ((m <pause-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Executer-srv:tree_id-val is deprecated.  Use Executer-srv:tree_id instead.")
  (tree_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pause-request>) ostream)
  "Serializes a message object of type '<pause-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tree_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tree_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pause-request>) istream)
  "Deserializes a message object of type '<pause-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pause-request>)))
  "Returns string type for a service object of type '<pause-request>"
  "Executer/pauseRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pause-request)))
  "Returns string type for a service object of type 'pause-request"
  "Executer/pauseRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pause-request>)))
  "Returns md5sum for a message object of type '<pause-request>"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pause-request)))
  "Returns md5sum for a message object of type 'pause-request"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pause-request>)))
  "Returns full string definition for message of type '<pause-request>"
  (cl:format cl:nil "string tree_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pause-request)))
  "Returns full string definition for message of type 'pause-request"
  (cl:format cl:nil "string tree_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pause-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tree_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pause-request>))
  "Converts a ROS message object to a list"
  (cl:list 'pause-request
    (cl:cons ':tree_id (tree_id msg))
))
;//! \htmlinclude pause-response.msg.html

(cl:defclass <pause-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass pause-response (<pause-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pause-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pause-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Executer-srv:<pause-response> is deprecated: use Executer-srv:pause-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <pause-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Executer-srv:output-val is deprecated.  Use Executer-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pause-response>) ostream)
  "Serializes a message object of type '<pause-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pause-response>) istream)
  "Deserializes a message object of type '<pause-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pause-response>)))
  "Returns string type for a service object of type '<pause-response>"
  "Executer/pauseResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pause-response)))
  "Returns string type for a service object of type 'pause-response"
  "Executer/pauseResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pause-response>)))
  "Returns md5sum for a message object of type '<pause-response>"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pause-response)))
  "Returns md5sum for a message object of type 'pause-response"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pause-response>)))
  "Returns full string definition for message of type '<pause-response>"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pause-response)))
  "Returns full string definition for message of type 'pause-response"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pause-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pause-response>))
  "Converts a ROS message object to a list"
  (cl:list 'pause-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'pause)))
  'pause-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'pause)))
  'pause-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pause)))
  "Returns string type for a service object of type '<pause>"
  "Executer/pause")