; Auto-generated. Do not edit!


(cl:in-package C11_OperatorControl-srv)


;//! \htmlinclude C11-request.msg.html

(cl:defclass <C11-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass C11-request (<C11-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_OperatorControl-srv:<C11-request> is deprecated: use C11_OperatorControl-srv:C11-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11-request>) ostream)
  "Serializes a message object of type '<C11-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11-request>) istream)
  "Deserializes a message object of type '<C11-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11-request>)))
  "Returns string type for a service object of type '<C11-request>"
  "C11_OperatorControl/C11Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11-request)))
  "Returns string type for a service object of type 'C11-request"
  "C11_OperatorControl/C11Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11-request>)))
  "Returns md5sum for a message object of type '<C11-request>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11-request)))
  "Returns md5sum for a message object of type 'C11-request"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11-request>)))
  "Returns full string definition for message of type '<C11-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11-request)))
  "Returns full string definition for message of type 'C11-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C11-request
))
;//! \htmlinclude C11-response.msg.html

(cl:defclass <C11-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass C11-response (<C11-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C11-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C11-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_OperatorControl-srv:<C11-response> is deprecated: use C11_OperatorControl-srv:C11-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C11-response>) ostream)
  "Serializes a message object of type '<C11-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C11-response>) istream)
  "Deserializes a message object of type '<C11-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C11-response>)))
  "Returns string type for a service object of type '<C11-response>"
  "C11_OperatorControl/C11Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11-response)))
  "Returns string type for a service object of type 'C11-response"
  "C11_OperatorControl/C11Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C11-response>)))
  "Returns md5sum for a message object of type '<C11-response>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C11-response)))
  "Returns md5sum for a message object of type 'C11-response"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C11-response>)))
  "Returns full string definition for message of type '<C11-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C11-response)))
  "Returns full string definition for message of type 'C11-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C11-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C11-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C11-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C11)))
  'C11-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C11)))
  'C11-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C11)))
  "Returns string type for a service object of type '<C11>"
  "C11_OperatorControl/C11")