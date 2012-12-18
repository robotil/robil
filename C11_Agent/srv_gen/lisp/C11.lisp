; Auto-generated. Do not edit!


(cl:in-package C11_Agent-srv)


;//! \htmlinclude C11-request.msg.html

(cl:defclass <C11-request> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type C11_Agent-msg:C11C32_PATH
    :initform (cl:make-instance 'C11_Agent-msg:C11C32_PATH)))
)

(cl:defclass C11-request (<C11-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<C11-request> is deprecated: use C11_Agent-srv:C11-request instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <C11-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:path-val is deprecated.  Use C11_Agent-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11-request>) ostream)
  "Serializes a message object of type '<C11-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11-request>) istream)
  "Deserializes a message object of type '<C11-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11-request>)))
  "Returns string type for a service object of type '<C11-request>"
  "C11_Agent/C11Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11-request)))
  "Returns string type for a service object of type 'C11-request"
  "C11_Agent/C11Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11-request>)))
  "Returns md5sum for a message object of type '<C11-request>"
  "ca160abea4d8358e11f4d9c2c371638e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11-request)))
  "Returns md5sum for a message object of type 'C11-request"
  "ca160abea4d8358e11f4d9c2c371638e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11-request>)))
  "Returns full string definition for message of type '<C11-request>"
  (cl:format cl:nil "~%C11_Agent/C11C32_PATH  path~%~%================================================================================~%MSG: C11_Agent/C11C32_PATH~%float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11-request)))
  "Returns full string definition for message of type 'C11-request"
  (cl:format cl:nil "~%C11_Agent/C11C32_PATH  path~%~%================================================================================~%MSG: C11_Agent/C11C32_PATH~%float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C11-request
    (cl:cons ':path (path msg))
))
;//! \htmlinclude C11-response.msg.html

(cl:defclass <C11-response> (roslisp-msg-protocol:ros-message)
  ((required_stt
    :reader required_stt
    :initarg :required_stt
    :type C11_Agent-msg:C34C11_STT
    :initform (cl:make-instance 'C11_Agent-msg:C34C11_STT))
   (path
    :reader path
    :initarg :path
    :type C11_Agent-msg:C32C11_PATH
    :initform (cl:make-instance 'C11_Agent-msg:C32C11_PATH)))
)

(cl:defclass C11-response (<C11-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<C11-response> is deprecated: use C11_Agent-srv:C11-response instead.")))

(cl:ensure-generic-function 'required_stt-val :lambda-list '(m))
(cl:defmethod required_stt-val ((m <C11-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:required_stt-val is deprecated.  Use C11_Agent-srv:required_stt instead.")
  (required_stt m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <C11-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:path-val is deprecated.  Use C11_Agent-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11-response>) ostream)
  "Serializes a message object of type '<C11-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'required_stt) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11-response>) istream)
  "Deserializes a message object of type '<C11-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'required_stt) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11-response>)))
  "Returns string type for a service object of type '<C11-response>"
  "C11_Agent/C11Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11-response)))
  "Returns string type for a service object of type 'C11-response"
  "C11_Agent/C11Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11-response>)))
  "Returns md5sum for a message object of type '<C11-response>"
  "ca160abea4d8358e11f4d9c2c371638e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11-response)))
  "Returns md5sum for a message object of type 'C11-response"
  "ca160abea4d8358e11f4d9c2c371638e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11-response>)))
  "Returns full string definition for message of type '<C11-response>"
  (cl:format cl:nil "C11_Agent/C34C11_STT  required_stt~%C11_Agent/C32C11_PATH path~%~%~%================================================================================~%MSG: C11_Agent/C34C11_STT~%int8 stt~%~%~%================================================================================~%MSG: C11_Agent/C32C11_PATH~%float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11-response)))
  "Returns full string definition for message of type 'C11-response"
  (cl:format cl:nil "C11_Agent/C34C11_STT  required_stt~%C11_Agent/C32C11_PATH path~%~%~%================================================================================~%MSG: C11_Agent/C34C11_STT~%int8 stt~%~%~%================================================================================~%MSG: C11_Agent/C32C11_PATH~%float64 lat~%float64 lon~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'required_stt))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C11-response
    (cl:cons ':required_stt (required_stt msg))
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C11)))
  'C11-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C11)))
  'C11-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11)))
  "Returns string type for a service object of type '<C11>"
  "C11_Agent/C11")