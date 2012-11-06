; Auto-generated. Do not edit!


(cl:in-package c34_Executer-srv)


;//! \htmlinclude run-request.msg.html

(cl:defclass <run-request> (roslisp-msg-protocol:ros-message)
  ((tree_id
    :reader tree_id
    :initarg :tree_id
    :type cl:string
    :initform "")
   (filename
    :reader filename
    :initarg :filename
    :type cl:string
    :initform ""))
)

(cl:defclass run-request (<run-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <run-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'run-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name c34_Executer-srv:<run-request> is deprecated: use c34_Executer-srv:run-request instead.")))

(cl:ensure-generic-function 'tree_id-val :lambda-list '(m))
(cl:defmethod tree_id-val ((m <run-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader c34_Executer-srv:tree_id-val is deprecated.  Use c34_Executer-srv:tree_id instead.")
  (tree_id m))

(cl:ensure-generic-function 'filename-val :lambda-list '(m))
(cl:defmethod filename-val ((m <run-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader c34_Executer-srv:filename-val is deprecated.  Use c34_Executer-srv:filename instead.")
  (filename m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <run-request>) ostream)
  "Serializes a message object of type '<run-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'tree_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'tree_id))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'filename))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'filename))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <run-request>) istream)
  "Deserializes a message object of type '<run-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'tree_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'tree_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<run-request>)))
  "Returns string type for a service object of type '<run-request>"
  "c34_Executer/runRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'run-request)))
  "Returns string type for a service object of type 'run-request"
  "c34_Executer/runRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<run-request>)))
  "Returns md5sum for a message object of type '<run-request>"
  "8f9adb19675774072202d219ae611673")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'run-request)))
  "Returns md5sum for a message object of type 'run-request"
  "8f9adb19675774072202d219ae611673")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<run-request>)))
  "Returns full string definition for message of type '<run-request>"
  (cl:format cl:nil "string tree_id~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'run-request)))
  "Returns full string definition for message of type 'run-request"
  (cl:format cl:nil "string tree_id~%string filename~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <run-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'tree_id))
     4 (cl:length (cl:slot-value msg 'filename))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <run-request>))
  "Converts a ROS message object to a list"
  (cl:list 'run-request
    (cl:cons ':tree_id (tree_id msg))
    (cl:cons ':filename (filename msg))
))
;//! \htmlinclude run-response.msg.html

(cl:defclass <run-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass run-response (<run-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <run-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'run-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name c34_Executer-srv:<run-response> is deprecated: use c34_Executer-srv:run-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <run-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader c34_Executer-srv:output-val is deprecated.  Use c34_Executer-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <run-response>) ostream)
  "Serializes a message object of type '<run-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <run-response>) istream)
  "Deserializes a message object of type '<run-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<run-response>)))
  "Returns string type for a service object of type '<run-response>"
  "c34_Executer/runResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'run-response)))
  "Returns string type for a service object of type 'run-response"
  "c34_Executer/runResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<run-response>)))
  "Returns md5sum for a message object of type '<run-response>"
  "8f9adb19675774072202d219ae611673")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'run-response)))
  "Returns md5sum for a message object of type 'run-response"
  "8f9adb19675774072202d219ae611673")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<run-response>)))
  "Returns full string definition for message of type '<run-response>"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'run-response)))
  "Returns full string definition for message of type 'run-response"
  (cl:format cl:nil "string output~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <run-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <run-response>))
  "Converts a ROS message object to a list"
  (cl:list 'run-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'run)))
  'run-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'run)))
  'run-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'run)))
  "Returns string type for a service object of type '<run>"
  "c34_Executer/run")