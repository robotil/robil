; Auto-generated. Do not edit!


(cl:in-package c34_Executer-srv)


;//! \htmlinclude resume-request.msg.html

(cl:defclass <resume-request> (roslisp-msg-protocol:ros-message)
  ((tree_id
    :reader tree_id
    :initarg :tree_id
    :type cl:string
    :initform ""))
)

(cl:defclass resume-request (<resume-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <resume-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'resume-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name c34_Executer-srv:<resume-request> is deprecated: use c34_Executer-srv:resume-request instead.")))

(cl:ensure-generic-function 'tree_id-val :lambda-list '(m))
(cl:defmethod tree_id-val ((m <resume-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader c34_Executer-srv:tree_id-val is deprecated.  Use c34_Executer-srv:tree_id instead.")
  (tree_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <resume-request>) ostream)
  "Serializes a message object of type '<resume-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tree_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tree_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <resume-request>) istream)
  "Deserializes a message object of type '<resume-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<resume-request>)))
  "Returns string type for a service object of type '<resume-request>"
  "c34_Executer/resumeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resume-request)))
  "Returns string type for a service object of type 'resume-request"
  "c34_Executer/resumeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<resume-request>)))
  "Returns md5sum for a message object of type '<resume-request>"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'resume-request)))
  "Returns md5sum for a message object of type 'resume-request"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<resume-request>)))
  "Returns full string definition for message of type '<resume-request>"
  (cl:format cl:nil "string tree_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'resume-request)))
  "Returns full string definition for message of type 'resume-request"
  (cl:format cl:nil "string tree_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <resume-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tree_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <resume-request>))
  "Converts a ROS message object to a list"
  (cl:list 'resume-request
    (cl:cons ':tree_id (tree_id msg))
))
;//! \htmlinclude resume-response.msg.html

(cl:defclass <resume-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass resume-response (<resume-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <resume-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'resume-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name c34_Executer-srv:<resume-response> is deprecated: use c34_Executer-srv:resume-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <resume-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader c34_Executer-srv:output-val is deprecated.  Use c34_Executer-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <resume-response>) ostream)
  "Serializes a message object of type '<resume-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <resume-response>) istream)
  "Deserializes a message object of type '<resume-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<resume-response>)))
  "Returns string type for a service object of type '<resume-response>"
  "c34_Executer/resumeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resume-response)))
  "Returns string type for a service object of type 'resume-response"
  "c34_Executer/resumeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<resume-response>)))
  "Returns md5sum for a message object of type '<resume-response>"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'resume-response)))
  "Returns md5sum for a message object of type 'resume-response"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<resume-response>)))
  "Returns full string definition for message of type '<resume-response>"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'resume-response)))
  "Returns full string definition for message of type 'resume-response"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <resume-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <resume-response>))
  "Converts a ROS message object to a list"
  (cl:list 'resume-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'resume)))
  'resume-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'resume)))
  'resume-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'resume)))
  "Returns string type for a service object of type '<resume>"
  "c34_Executer/resume")