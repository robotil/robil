; Auto-generated. Do not edit!


(cl:in-package c34_Executer-srv)


;//! \htmlinclude show_table_msg-request.msg.html

(cl:defclass <show_table_msg-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass show_table_msg-request (<show_table_msg-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <show_table_msg-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'show_table_msg-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name c34_Executer-srv:<show_table_msg-request> is deprecated: use c34_Executer-srv:show_table_msg-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <show_table_msg-request>) ostream)
  "Serializes a message object of type '<show_table_msg-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <show_table_msg-request>) istream)
  "Deserializes a message object of type '<show_table_msg-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<show_table_msg-request>)))
  "Returns string type for a service object of type '<show_table_msg-request>"
  "c34_Executer/show_table_msgRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'show_table_msg-request)))
  "Returns string type for a service object of type 'show_table_msg-request"
  "c34_Executer/show_table_msgRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<show_table_msg-request>)))
  "Returns md5sum for a message object of type '<show_table_msg-request>"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'show_table_msg-request)))
  "Returns md5sum for a message object of type 'show_table_msg-request"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<show_table_msg-request>)))
  "Returns full string definition for message of type '<show_table_msg-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'show_table_msg-request)))
  "Returns full string definition for message of type 'show_table_msg-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <show_table_msg-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <show_table_msg-request>))
  "Converts a ROS message object to a list"
  (cl:list 'show_table_msg-request
))
;//! \htmlinclude show_table_msg-response.msg.html

(cl:defclass <show_table_msg-response> (roslisp-msg-protocol:ros-message)
  ((content
    :reader content
    :initarg :content
    :type cl:string
    :initform ""))
)

(cl:defclass show_table_msg-response (<show_table_msg-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <show_table_msg-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'show_table_msg-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name c34_Executer-srv:<show_table_msg-response> is deprecated: use c34_Executer-srv:show_table_msg-response instead.")))

(cl:ensure-generic-function 'content-val :lambda-list '(m))
(cl:defmethod content-val ((m <show_table_msg-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader c34_Executer-srv:content-val is deprecated.  Use c34_Executer-srv:content instead.")
  (content m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <show_table_msg-response>) ostream)
  "Serializes a message object of type '<show_table_msg-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'content))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'content))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <show_table_msg-response>) istream)
  "Deserializes a message object of type '<show_table_msg-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<show_table_msg-response>)))
  "Returns string type for a service object of type '<show_table_msg-response>"
  "c34_Executer/show_table_msgResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'show_table_msg-response)))
  "Returns string type for a service object of type 'show_table_msg-response"
  "c34_Executer/show_table_msgResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<show_table_msg-response>)))
  "Returns md5sum for a message object of type '<show_table_msg-response>"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'show_table_msg-response)))
  "Returns md5sum for a message object of type 'show_table_msg-response"
  "c2e84951ee6d0addf437bfddd5b19734")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<show_table_msg-response>)))
  "Returns full string definition for message of type '<show_table_msg-response>"
  (cl:format cl:nil "string content~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'show_table_msg-response)))
  "Returns full string definition for message of type 'show_table_msg-response"
  (cl:format cl:nil "string content~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <show_table_msg-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'content))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <show_table_msg-response>))
  "Converts a ROS message object to a list"
  (cl:list 'show_table_msg-response
    (cl:cons ':content (content msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'show_table_msg)))
  'show_table_msg-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'show_table_msg)))
  'show_table_msg-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'show_table_msg)))
  "Returns string type for a service object of type '<show_table_msg>"
  "c34_Executer/show_table_msg")