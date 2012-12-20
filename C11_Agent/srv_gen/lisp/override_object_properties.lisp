; Auto-generated. Do not edit!


(cl:in-package C11_Agent-srv)


;//! \htmlinclude override_object_properties-request.msg.html

(cl:defclass <override_object_properties-request> (roslisp-msg-protocol:ros-message)
  ((obp
    :reader obp
    :initarg :obp
    :type C11_Agent-msg:C11C23_OBP
    :initform (cl:make-instance 'C11_Agent-msg:C11C23_OBP)))
)

(cl:defclass override_object_properties-request (<override_object_properties-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <override_object_properties-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'override_object_properties-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<override_object_properties-request> is deprecated: use C11_Agent-srv:override_object_properties-request instead.")))

(cl:ensure-generic-function 'obp-val :lambda-list '(m))
(cl:defmethod obp-val ((m <override_object_properties-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:obp-val is deprecated.  Use C11_Agent-srv:obp instead.")
  (obp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <override_object_properties-request>) ostream)
  "Serializes a message object of type '<override_object_properties-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obp) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <override_object_properties-request>) istream)
  "Deserializes a message object of type '<override_object_properties-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obp) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<override_object_properties-request>)))
  "Returns string type for a service object of type '<override_object_properties-request>"
  "C11_Agent/override_object_propertiesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'override_object_properties-request)))
  "Returns string type for a service object of type 'override_object_properties-request"
  "C11_Agent/override_object_propertiesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<override_object_properties-request>)))
  "Returns md5sum for a message object of type '<override_object_properties-request>"
  "80a5a708c1560a51133202325435fc54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'override_object_properties-request)))
  "Returns md5sum for a message object of type 'override_object_properties-request"
  "80a5a708c1560a51133202325435fc54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<override_object_properties-request>)))
  "Returns full string definition for message of type '<override_object_properties-request>"
  (cl:format cl:nil "C11_Agent/C11C23_OBP   obp~%~%~%================================================================================~%MSG: C11_Agent/C11C23_OBP~%int16 ACT~%int16 ACT_MODIFIED = 0~%int16 ACT_NEW      = 1~%int16 FRZ~%int16 FRZ_KEEP  = 0~%int16 ACT_RETRY = 1~%int16 ID~%string NAME~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/D3SPACE ORI~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%================================================================================~%MSG: C11_Agent/D3SPACE~%float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'override_object_properties-request)))
  "Returns full string definition for message of type 'override_object_properties-request"
  (cl:format cl:nil "C11_Agent/C11C23_OBP   obp~%~%~%================================================================================~%MSG: C11_Agent/C11C23_OBP~%int16 ACT~%int16 ACT_MODIFIED = 0~%int16 ACT_NEW      = 1~%int16 FRZ~%int16 FRZ_KEEP  = 0~%int16 ACT_RETRY = 1~%int16 ID~%string NAME~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/D3SPACE ORI~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%================================================================================~%MSG: C11_Agent/D3SPACE~%float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <override_object_properties-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obp))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <override_object_properties-request>))
  "Converts a ROS message object to a list"
  (cl:list 'override_object_properties-request
    (cl:cons ':obp (obp msg))
))
;//! \htmlinclude override_object_properties-response.msg.html

(cl:defclass <override_object_properties-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass override_object_properties-response (<override_object_properties-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <override_object_properties-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'override_object_properties-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<override_object_properties-response> is deprecated: use C11_Agent-srv:override_object_properties-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <override_object_properties-response>) ostream)
  "Serializes a message object of type '<override_object_properties-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <override_object_properties-response>) istream)
  "Deserializes a message object of type '<override_object_properties-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<override_object_properties-response>)))
  "Returns string type for a service object of type '<override_object_properties-response>"
  "C11_Agent/override_object_propertiesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'override_object_properties-response)))
  "Returns string type for a service object of type 'override_object_properties-response"
  "C11_Agent/override_object_propertiesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<override_object_properties-response>)))
  "Returns md5sum for a message object of type '<override_object_properties-response>"
  "80a5a708c1560a51133202325435fc54")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'override_object_properties-response)))
  "Returns md5sum for a message object of type 'override_object_properties-response"
  "80a5a708c1560a51133202325435fc54")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<override_object_properties-response>)))
  "Returns full string definition for message of type '<override_object_properties-response>"
  (cl:format cl:nil "~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'override_object_properties-response)))
  "Returns full string definition for message of type 'override_object_properties-response"
  (cl:format cl:nil "~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <override_object_properties-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <override_object_properties-response>))
  "Converts a ROS message object to a list"
  (cl:list 'override_object_properties-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'override_object_properties)))
  'override_object_properties-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'override_object_properties)))
  'override_object_properties-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'override_object_properties)))
  "Returns string type for a service object of type '<override_object_properties>"
  "C11_Agent/override_object_properties")