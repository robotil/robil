; Auto-generated. Do not edit!


(cl:in-package C34_Executer-srv)


;//! \htmlinclude step-request.msg.html

(cl:defclass <step-request> (roslisp-msg-protocol:ros-message)
  ((tree_id
    :reader tree_id
    :initarg :tree_id
    :type cl:string
    :initform ""))
)

(cl:defclass step-request (<step-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <step-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'step-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<step-request> is deprecated: use C34_Executer-srv:step-request instead.")))

(cl:ensure-generic-function 'tree_id-val :lambda-list '(m))
(cl:defmethod tree_id-val ((m <step-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:tree_id-val is deprecated.  Use C34_Executer-srv:tree_id instead.")
  (tree_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <step-request>) ostream)
  "Serializes a message object of type '<step-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tree_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tree_id))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <step-request>) istream)
  "Deserializes a message object of type '<step-request>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<step-request>)))
  "Returns string type for a service object of type '<step-request>"
  "C34_Executer/stepRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'step-request)))
  "Returns string type for a service object of type 'step-request"
  "C34_Executer/stepRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<step-request>)))
  "Returns md5sum for a message object of type '<step-request>"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'step-request)))
  "Returns md5sum for a message object of type 'step-request"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<step-request>)))
  "Returns full string definition for message of type '<step-request>"
  (cl:format cl:nil "string tree_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'step-request)))
  "Returns full string definition for message of type 'step-request"
  (cl:format cl:nil "string tree_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <step-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tree_id))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <step-request>))
  "Converts a ROS message object to a list"
  (cl:list 'step-request
    (cl:cons ':tree_id (tree_id msg))
))
;//! \htmlinclude step-response.msg.html

(cl:defclass <step-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass step-response (<step-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <step-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'step-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<step-response> is deprecated: use C34_Executer-srv:step-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <step-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:output-val is deprecated.  Use C34_Executer-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <step-response>) ostream)
  "Serializes a message object of type '<step-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <step-response>) istream)
  "Deserializes a message object of type '<step-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<step-response>)))
  "Returns string type for a service object of type '<step-response>"
  "C34_Executer/stepResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'step-response)))
  "Returns string type for a service object of type 'step-response"
  "C34_Executer/stepResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<step-response>)))
  "Returns md5sum for a message object of type '<step-response>"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'step-response)))
  "Returns md5sum for a message object of type 'step-response"
  "2d787918d0efb6823e526f109f8fafe3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<step-response>)))
  "Returns full string definition for message of type '<step-response>"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'step-response)))
  "Returns full string definition for message of type 'step-response"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <step-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <step-response>))
  "Converts a ROS message object to a list"
  (cl:list 'step-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'step)))
  'step-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'step)))
  'step-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'step)))
  "Returns string type for a service object of type '<step>"
  "C34_Executer/step")