; Auto-generated. Do not edit!


(cl:in-package C34_Executer-srv)


;//! \htmlinclude whoIsRunning-request.msg.html

(cl:defclass <whoIsRunning-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass whoIsRunning-request (<whoIsRunning-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <whoIsRunning-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'whoIsRunning-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<whoIsRunning-request> is deprecated: use C34_Executer-srv:whoIsRunning-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <whoIsRunning-request>) ostream)
  "Serializes a message object of type '<whoIsRunning-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <whoIsRunning-request>) istream)
  "Deserializes a message object of type '<whoIsRunning-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<whoIsRunning-request>)))
  "Returns string type for a service object of type '<whoIsRunning-request>"
  "C34_Executer/whoIsRunningRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'whoIsRunning-request)))
  "Returns string type for a service object of type 'whoIsRunning-request"
  "C34_Executer/whoIsRunningRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<whoIsRunning-request>)))
  "Returns md5sum for a message object of type '<whoIsRunning-request>"
  "3195cffe705fdcf5e499ae7b27e958e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'whoIsRunning-request)))
  "Returns md5sum for a message object of type 'whoIsRunning-request"
  "3195cffe705fdcf5e499ae7b27e958e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<whoIsRunning-request>)))
  "Returns full string definition for message of type '<whoIsRunning-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'whoIsRunning-request)))
  "Returns full string definition for message of type 'whoIsRunning-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <whoIsRunning-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <whoIsRunning-request>))
  "Converts a ROS message object to a list"
  (cl:list 'whoIsRunning-request
))
;//! \htmlinclude whoIsRunning-response.msg.html

(cl:defclass <whoIsRunning-response> (roslisp-msg-protocol:ros-message)
  ((runningList
    :reader runningList
    :initarg :runningList
    :type cl:string
    :initform ""))
)

(cl:defclass whoIsRunning-response (<whoIsRunning-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <whoIsRunning-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'whoIsRunning-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<whoIsRunning-response> is deprecated: use C34_Executer-srv:whoIsRunning-response instead.")))

(cl:ensure-generic-function 'runningList-val :lambda-list '(m))
(cl:defmethod runningList-val ((m <whoIsRunning-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:runningList-val is deprecated.  Use C34_Executer-srv:runningList instead.")
  (runningList m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <whoIsRunning-response>) ostream)
  "Serializes a message object of type '<whoIsRunning-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'runningList))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'runningList))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <whoIsRunning-response>) istream)
  "Deserializes a message object of type '<whoIsRunning-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'runningList) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'runningList) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<whoIsRunning-response>)))
  "Returns string type for a service object of type '<whoIsRunning-response>"
  "C34_Executer/whoIsRunningResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'whoIsRunning-response)))
  "Returns string type for a service object of type 'whoIsRunning-response"
  "C34_Executer/whoIsRunningResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<whoIsRunning-response>)))
  "Returns md5sum for a message object of type '<whoIsRunning-response>"
  "3195cffe705fdcf5e499ae7b27e958e1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'whoIsRunning-response)))
  "Returns md5sum for a message object of type 'whoIsRunning-response"
  "3195cffe705fdcf5e499ae7b27e958e1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<whoIsRunning-response>)))
  "Returns full string definition for message of type '<whoIsRunning-response>"
  (cl:format cl:nil "string runningList~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'whoIsRunning-response)))
  "Returns full string definition for message of type 'whoIsRunning-response"
  (cl:format cl:nil "string runningList~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <whoIsRunning-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'runningList))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <whoIsRunning-response>))
  "Converts a ROS message object to a list"
  (cl:list 'whoIsRunning-response
    (cl:cons ':runningList (runningList msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'whoIsRunning)))
  'whoIsRunning-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'whoIsRunning)))
  'whoIsRunning-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'whoIsRunning)))
  "Returns string type for a service object of type '<whoIsRunning>"
  "C34_Executer/whoIsRunning")