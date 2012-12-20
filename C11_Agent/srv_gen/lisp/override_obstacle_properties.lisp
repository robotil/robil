; Auto-generated. Do not edit!


(cl:in-package C11_Agent-srv)


;//! \htmlinclude override_obstacle_properties-request.msg.html

(cl:defclass <override_obstacle_properties-request> (roslisp-msg-protocol:ros-message)
  ((osp
    :reader osp
    :initarg :osp
    :type C11_Agent-msg:C11C24_OSP
    :initform (cl:make-instance 'C11_Agent-msg:C11C24_OSP)))
)

(cl:defclass override_obstacle_properties-request (<override_obstacle_properties-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <override_obstacle_properties-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'override_obstacle_properties-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<override_obstacle_properties-request> is deprecated: use C11_Agent-srv:override_obstacle_properties-request instead.")))

(cl:ensure-generic-function 'osp-val :lambda-list '(m))
(cl:defmethod osp-val ((m <override_obstacle_properties-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:osp-val is deprecated.  Use C11_Agent-srv:osp instead.")
  (osp m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <override_obstacle_properties-request>) ostream)
  "Serializes a message object of type '<override_obstacle_properties-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'osp) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <override_obstacle_properties-request>) istream)
  "Deserializes a message object of type '<override_obstacle_properties-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'osp) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<override_obstacle_properties-request>)))
  "Returns string type for a service object of type '<override_obstacle_properties-request>"
  "C11_Agent/override_obstacle_propertiesRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'override_obstacle_properties-request)))
  "Returns string type for a service object of type 'override_obstacle_properties-request"
  "C11_Agent/override_obstacle_propertiesRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<override_obstacle_properties-request>)))
  "Returns md5sum for a message object of type '<override_obstacle_properties-request>"
  "86960dd616b9dfcf6d09f709fea7366c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'override_obstacle_properties-request)))
  "Returns md5sum for a message object of type 'override_obstacle_properties-request"
  "86960dd616b9dfcf6d09f709fea7366c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<override_obstacle_properties-request>)))
  "Returns full string definition for message of type '<override_obstacle_properties-request>"
  (cl:format cl:nil "C11_Agent/C11C24_OSP   osp~%~%================================================================================~%MSG: C11_Agent/C11C24_OSP~%int32 ACT_TYPE~%int32 ACT_TYPE_MODIFIED = 0~%int32 ACT_TYPE_DELETED  = 1~%int32 ACT_TYPE_NEW      = 2~%int32 FRZ~%int32 FRZ_KEEP  = 0~%int32 FRZ_RETRY = 1~%int32 TYPE~%int32 TYPE_STATIC=0~%int32 TYPE_DYNAMIC=1~%int16 ID~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/pathLocation[] IGNR~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'override_obstacle_properties-request)))
  "Returns full string definition for message of type 'override_obstacle_properties-request"
  (cl:format cl:nil "C11_Agent/C11C24_OSP   osp~%~%================================================================================~%MSG: C11_Agent/C11C24_OSP~%int32 ACT_TYPE~%int32 ACT_TYPE_MODIFIED = 0~%int32 ACT_TYPE_DELETED  = 1~%int32 ACT_TYPE_NEW      = 2~%int32 FRZ~%int32 FRZ_KEEP  = 0~%int32 FRZ_RETRY = 1~%int32 TYPE~%int32 TYPE_STATIC=0~%int32 TYPE_DYNAMIC=1~%int16 ID~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/pathLocation[] IGNR~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <override_obstacle_properties-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'osp))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <override_obstacle_properties-request>))
  "Converts a ROS message object to a list"
  (cl:list 'override_obstacle_properties-request
    (cl:cons ':osp (osp msg))
))
;//! \htmlinclude override_obstacle_properties-response.msg.html

(cl:defclass <override_obstacle_properties-response> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass override_obstacle_properties-response (<override_obstacle_properties-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <override_obstacle_properties-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'override_obstacle_properties-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<override_obstacle_properties-response> is deprecated: use C11_Agent-srv:override_obstacle_properties-response instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <override_obstacle_properties-response>) ostream)
  "Serializes a message object of type '<override_obstacle_properties-response>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <override_obstacle_properties-response>) istream)
  "Deserializes a message object of type '<override_obstacle_properties-response>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<override_obstacle_properties-response>)))
  "Returns string type for a service object of type '<override_obstacle_properties-response>"
  "C11_Agent/override_obstacle_propertiesResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'override_obstacle_properties-response)))
  "Returns string type for a service object of type 'override_obstacle_properties-response"
  "C11_Agent/override_obstacle_propertiesResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<override_obstacle_properties-response>)))
  "Returns md5sum for a message object of type '<override_obstacle_properties-response>"
  "86960dd616b9dfcf6d09f709fea7366c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'override_obstacle_properties-response)))
  "Returns md5sum for a message object of type 'override_obstacle_properties-response"
  "86960dd616b9dfcf6d09f709fea7366c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<override_obstacle_properties-response>)))
  "Returns full string definition for message of type '<override_obstacle_properties-response>"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'override_obstacle_properties-response)))
  "Returns full string definition for message of type 'override_obstacle_properties-response"
  (cl:format cl:nil "~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <override_obstacle_properties-response>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <override_obstacle_properties-response>))
  "Converts a ROS message object to a list"
  (cl:list 'override_obstacle_properties-response
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'override_obstacle_properties)))
  'override_obstacle_properties-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'override_obstacle_properties)))
  'override_obstacle_properties-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'override_obstacle_properties)))
  "Returns string type for a service object of type '<override_obstacle_properties>"
  "C11_Agent/override_obstacle_properties")