; Auto-generated. Do not edit!


(cl:in-package C34_Executer-srv)


;//! \htmlinclude ls-request.msg.html

(cl:defclass <ls-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass ls-request (<ls-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ls-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ls-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<ls-request> is deprecated: use C34_Executer-srv:ls-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ls-request>) ostream)
  "Serializes a message object of type '<ls-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ls-request>) istream)
  "Deserializes a message object of type '<ls-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ls-request>)))
  "Returns string type for a service object of type '<ls-request>"
  "C34_Executer/lsRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ls-request)))
  "Returns string type for a service object of type 'ls-request"
  "C34_Executer/lsRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ls-request>)))
  "Returns md5sum for a message object of type '<ls-request>"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ls-request)))
  "Returns md5sum for a message object of type 'ls-request"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ls-request>)))
  "Returns full string definition for message of type '<ls-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ls-request)))
  "Returns full string definition for message of type 'ls-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ls-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ls-request>))
  "Converts a ROS message object to a list"
  (cl:list 'ls-request
))
;//! \htmlinclude ls-response.msg.html

(cl:defclass <ls-response> (roslisp-msg-protocol:ros-message)
  ((content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass ls-response (<ls-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ls-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ls-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C34_Executer-srv:<ls-response> is deprecated: use C34_Executer-srv:ls-response instead.")))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <ls-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C34_Executer-srv:content-val is deprecated.  Use C34_Executer-srv:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ls-response>) ostream)
  "Serializes a message object of type '<ls-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ls-response>) istream)
  "Deserializes a message object of type '<ls-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ls-response>)))
  "Returns string type for a service object of type '<ls-response>"
  "C34_Executer/lsResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ls-response)))
  "Returns string type for a service object of type 'ls-response"
  "C34_Executer/lsResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ls-response>)))
  "Returns md5sum for a message object of type '<ls-response>"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ls-response)))
  "Returns md5sum for a message object of type 'ls-response"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ls-response>)))
  "Returns full string definition for message of type '<ls-response>"
  (cl:format cl:nil "string content~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ls-response)))
  "Returns full string definition for message of type 'ls-response"
  (cl:format cl:nil "string content~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ls-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ls-response>))
  "Converts a ROS message object to a list"
  (cl:list 'ls-response
    (cl:cons ':content (content msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'ls)))
  'ls-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'ls)))
  'ls-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ls)))
  "Returns string type for a service object of type '<ls>"
  "C34_Executer/ls")