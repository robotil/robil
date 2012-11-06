; Auto-generated. Do not edit!


(cl:in-package Executer-srv)


;//! \htmlinclude cd-request.msg.html

(cl:defclass <cd-request> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type cl:string
    :initform ""))
)

(cl:defclass cd-request (<cd-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cd-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cd-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Executer-srv:<cd-request> is deprecated: use Executer-srv:cd-request instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <cd-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Executer-srv:path-val is deprecated.  Use Executer-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cd-request>) ostream)
  "Serializes a message object of type '<cd-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cd-request>) istream)
  "Deserializes a message object of type '<cd-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'path) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'path) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cd-request>)))
  "Returns string type for a service object of type '<cd-request>"
  "Executer/cdRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cd-request)))
  "Returns string type for a service object of type 'cd-request"
  "Executer/cdRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cd-request>)))
  "Returns md5sum for a message object of type '<cd-request>"
  "384058fb0d9614cd7c9170628fd63f12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cd-request)))
  "Returns md5sum for a message object of type 'cd-request"
  "384058fb0d9614cd7c9170628fd63f12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cd-request>)))
  "Returns full string definition for message of type '<cd-request>"
  (cl:format cl:nil "string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cd-request)))
  "Returns full string definition for message of type 'cd-request"
  (cl:format cl:nil "string path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cd-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cd-request>))
  "Converts a ROS message object to a list"
  (cl:list 'cd-request
    (cl:cons ':path (path msg))
))
;//! \htmlinclude cd-response.msg.html

(cl:defclass <cd-response> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type cl:string
    :initform ""))
)

(cl:defclass cd-response (<cd-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <cd-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'cd-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name Executer-srv:<cd-response> is deprecated: use Executer-srv:cd-response instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <cd-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader Executer-srv:location-val is deprecated.  Use Executer-srv:location instead.")
  (location m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <cd-response>) ostream)
  "Serializes a message object of type '<cd-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'location))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'location))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <cd-response>) istream)
  "Deserializes a message object of type '<cd-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'location) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'location) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<cd-response>)))
  "Returns string type for a service object of type '<cd-response>"
  "Executer/cdResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cd-response)))
  "Returns string type for a service object of type 'cd-response"
  "Executer/cdResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<cd-response>)))
  "Returns md5sum for a message object of type '<cd-response>"
  "384058fb0d9614cd7c9170628fd63f12")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'cd-response)))
  "Returns md5sum for a message object of type 'cd-response"
  "384058fb0d9614cd7c9170628fd63f12")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<cd-response>)))
  "Returns full string definition for message of type '<cd-response>"
  (cl:format cl:nil "string location~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'cd-response)))
  "Returns full string definition for message of type 'cd-response"
  (cl:format cl:nil "string location~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <cd-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'location))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <cd-response>))
  "Converts a ROS message object to a list"
  (cl:list 'cd-response
    (cl:cons ':location (location msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'cd)))
  'cd-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'cd)))
  'cd-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'cd)))
  "Returns string type for a service object of type '<cd>"
  "Executer/cd")