; Auto-generated. Do not edit!


(cl:in-package C11_Agent-srv)


;//! \htmlinclude object_map-request.msg.html

(cl:defclass <object_map-request> (roslisp-msg-protocol:ros-message)
  ((obm
    :reader obm
    :initarg :obm
    :type C11_Agent-msg:C11C23_OBM
    :initform (cl:make-instance 'C11_Agent-msg:C11C23_OBM)))
)

(cl:defclass object_map-request (<object_map-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <object_map-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'object_map-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<object_map-request> is deprecated: use C11_Agent-srv:object_map-request instead.")))

(cl:ensure-generic-function 'obm-val :lambda-list '(m))
(cl:defmethod obm-val ((m <object_map-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:obm-val is deprecated.  Use C11_Agent-srv:obm instead.")
  (obm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <object_map-request>) ostream)
  "Serializes a message object of type '<object_map-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obm) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <object_map-request>) istream)
  "Deserializes a message object of type '<object_map-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obm) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<object_map-request>)))
  "Returns string type for a service object of type '<object_map-request>"
  "C11_Agent/object_mapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object_map-request)))
  "Returns string type for a service object of type 'object_map-request"
  "C11_Agent/object_mapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<object_map-request>)))
  "Returns md5sum for a message object of type '<object_map-request>"
  "135a2212300a3065c3dd01227eabc7d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'object_map-request)))
  "Returns md5sum for a message object of type 'object_map-request"
  "135a2212300a3065c3dd01227eabc7d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<object_map-request>)))
  "Returns full string definition for message of type '<object_map-request>"
  (cl:format cl:nil "~%C11_Agent/C11C23_OBM  obm~%~%~%================================================================================~%MSG: C11_Agent/C11C23_OBM~%float64 LAT~%int16 SCN~%int16 SCN_SCAN=0~%int16 SCN_CURRENT=1~%int16 MOV~%int16 MOV_NONE     = 0~%int16 MOV_HEAD     = 1~%int16 MOV_POSTURE  = 2~%int16 MOV_POSITION = 3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'object_map-request)))
  "Returns full string definition for message of type 'object_map-request"
  (cl:format cl:nil "~%C11_Agent/C11C23_OBM  obm~%~%~%================================================================================~%MSG: C11_Agent/C11C23_OBM~%float64 LAT~%int16 SCN~%int16 SCN_SCAN=0~%int16 SCN_CURRENT=1~%int16 MOV~%int16 MOV_NONE     = 0~%int16 MOV_HEAD     = 1~%int16 MOV_POSTURE  = 2~%int16 MOV_POSITION = 3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <object_map-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obm))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <object_map-request>))
  "Converts a ROS message object to a list"
  (cl:list 'object_map-request
    (cl:cons ':obm (obm msg))
))
;//! \htmlinclude object_map-response.msg.html

(cl:defclass <object_map-response> (roslisp-msg-protocol:ros-message)
  ((obm
    :reader obm
    :initarg :obm
    :type C11_Agent-msg:C23C11_OSM
    :initform (cl:make-instance 'C11_Agent-msg:C23C11_OSM)))
)

(cl:defclass object_map-response (<object_map-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <object_map-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'object_map-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<object_map-response> is deprecated: use C11_Agent-srv:object_map-response instead.")))

(cl:ensure-generic-function 'obm-val :lambda-list '(m))
(cl:defmethod obm-val ((m <object_map-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:obm-val is deprecated.  Use C11_Agent-srv:obm instead.")
  (obm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <object_map-response>) ostream)
  "Serializes a message object of type '<object_map-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'obm) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <object_map-response>) istream)
  "Deserializes a message object of type '<object_map-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'obm) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<object_map-response>)))
  "Returns string type for a service object of type '<object_map-response>"
  "C11_Agent/object_mapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object_map-response)))
  "Returns string type for a service object of type 'object_map-response"
  "C11_Agent/object_mapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<object_map-response>)))
  "Returns md5sum for a message object of type '<object_map-response>"
  "135a2212300a3065c3dd01227eabc7d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'object_map-response)))
  "Returns md5sum for a message object of type 'object_map-response"
  "135a2212300a3065c3dd01227eabc7d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<object_map-response>)))
  "Returns full string definition for message of type '<object_map-response>"
  (cl:format cl:nil "C11_Agent/C23C11_OSM  obm~%~%~%================================================================================~%MSG: C11_Agent/C23C11_OSM~%int16 ID~%string NAME~%C11_Agent/coord EXTR_3D~%C11_Agent/coord CNTR~%C11_Agent/D3SPACE ORIN~%================================================================================~%MSG: C11_Agent/coord~%float64 X~%float64 Y~%float64 Z~%================================================================================~%MSG: C11_Agent/D3SPACE~%float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'object_map-response)))
  "Returns full string definition for message of type 'object_map-response"
  (cl:format cl:nil "C11_Agent/C23C11_OSM  obm~%~%~%================================================================================~%MSG: C11_Agent/C23C11_OSM~%int16 ID~%string NAME~%C11_Agent/coord EXTR_3D~%C11_Agent/coord CNTR~%C11_Agent/D3SPACE ORIN~%================================================================================~%MSG: C11_Agent/coord~%float64 X~%float64 Y~%float64 Z~%================================================================================~%MSG: C11_Agent/D3SPACE~%float64 ROLL~%float64 PITCH~%float64 YAW~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <object_map-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'obm))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <object_map-response>))
  "Converts a ROS message object to a list"
  (cl:list 'object_map-response
    (cl:cons ':obm (obm msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'object_map)))
  'object_map-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'object_map)))
  'object_map-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'object_map)))
  "Returns string type for a service object of type '<object_map>"
  "C11_Agent/object_map")