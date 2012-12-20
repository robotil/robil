; Auto-generated. Do not edit!


(cl:in-package C11_Agent-srv)


;//! \htmlinclude obstacle_map-request.msg.html

(cl:defclass <obstacle_map-request> (roslisp-msg-protocol:ros-message)
  ((osm
    :reader osm
    :initarg :osm
    :type C11_Agent-msg:C11C24_OSM
    :initform (cl:make-instance 'C11_Agent-msg:C11C24_OSM)))
)

(cl:defclass obstacle_map-request (<obstacle_map-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacle_map-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacle_map-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<obstacle_map-request> is deprecated: use C11_Agent-srv:obstacle_map-request instead.")))

(cl:ensure-generic-function 'osm-val :lambda-list '(m))
(cl:defmethod osm-val ((m <obstacle_map-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:osm-val is deprecated.  Use C11_Agent-srv:osm instead.")
  (osm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacle_map-request>) ostream)
  "Serializes a message object of type '<obstacle_map-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'osm) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacle_map-request>) istream)
  "Deserializes a message object of type '<obstacle_map-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'osm) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacle_map-request>)))
  "Returns string type for a service object of type '<obstacle_map-request>"
  "C11_Agent/obstacle_mapRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle_map-request)))
  "Returns string type for a service object of type 'obstacle_map-request"
  "C11_Agent/obstacle_mapRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacle_map-request>)))
  "Returns md5sum for a message object of type '<obstacle_map-request>"
  "9ec3dfc0c61d90cd4fc19bd886162877")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacle_map-request)))
  "Returns md5sum for a message object of type 'obstacle_map-request"
  "9ec3dfc0c61d90cd4fc19bd886162877")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacle_map-request>)))
  "Returns full string definition for message of type '<obstacle_map-request>"
  (cl:format cl:nil "~%C11_Agent/C11C24_OSM  osm~%~%~%================================================================================~%MSG: C11_Agent/C11C24_OSM~%int16 TYP~%int16 TYP_STATIC  = 0~%int16 TYP_DYNAMIC = 1~%float64 LAT~%int16 SCN~%int16 SCN_SCAN    = 0~%int16 SCN_CURRENT = 1~%int16 MOV~%int16 MOV_NONE     = 0~%int16 MOV_HEAD     = 1~%int16 MOV_POSTURE  = 2~%int16 MOV_POSITION = 3~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacle_map-request)))
  "Returns full string definition for message of type 'obstacle_map-request"
  (cl:format cl:nil "~%C11_Agent/C11C24_OSM  osm~%~%~%================================================================================~%MSG: C11_Agent/C11C24_OSM~%int16 TYP~%int16 TYP_STATIC  = 0~%int16 TYP_DYNAMIC = 1~%float64 LAT~%int16 SCN~%int16 SCN_SCAN    = 0~%int16 SCN_CURRENT = 1~%int16 MOV~%int16 MOV_NONE     = 0~%int16 MOV_HEAD     = 1~%int16 MOV_POSTURE  = 2~%int16 MOV_POSITION = 3~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacle_map-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'osm))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacle_map-request>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacle_map-request
    (cl:cons ':osm (osm msg))
))
;//! \htmlinclude obstacle_map-response.msg.html

(cl:defclass <obstacle_map-response> (roslisp-msg-protocol:ros-message)
  ((osm
    :reader osm
    :initarg :osm
    :type C11_Agent-msg:C24C11_OSM
    :initform (cl:make-instance 'C11_Agent-msg:C24C11_OSM)))
)

(cl:defclass obstacle_map-response (<obstacle_map-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <obstacle_map-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'obstacle_map-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C11_Agent-srv:<obstacle_map-response> is deprecated: use C11_Agent-srv:obstacle_map-response instead.")))

(cl:ensure-generic-function 'osm-val :lambda-list '(m))
(cl:defmethod osm-val ((m <obstacle_map-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C11_Agent-srv:osm-val is deprecated.  Use C11_Agent-srv:osm instead.")
  (osm m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <obstacle_map-response>) ostream)
  "Serializes a message object of type '<obstacle_map-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'osm) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <obstacle_map-response>) istream)
  "Deserializes a message object of type '<obstacle_map-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'osm) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<obstacle_map-response>)))
  "Returns string type for a service object of type '<obstacle_map-response>"
  "C11_Agent/obstacle_mapResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle_map-response)))
  "Returns string type for a service object of type 'obstacle_map-response"
  "C11_Agent/obstacle_mapResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<obstacle_map-response>)))
  "Returns md5sum for a message object of type '<obstacle_map-response>"
  "9ec3dfc0c61d90cd4fc19bd886162877")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'obstacle_map-response)))
  "Returns md5sum for a message object of type 'obstacle_map-response"
  "9ec3dfc0c61d90cd4fc19bd886162877")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<obstacle_map-response>)))
  "Returns full string definition for message of type '<obstacle_map-response>"
  (cl:format cl:nil "C11_Agent/C24C11_OSM  osm~%~%~%================================================================================~%MSG: C11_Agent/C24C11_OSM~%int32 TYPE~%int32 TYPE_STATIC  = 0~%int32 TYPE_DYNAMIC = 1~%int16 ID~%C11_Agent/coord[] COM~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/coord EXTR_3D~%================================================================================~%MSG: C11_Agent/coord~%float64 X~%float64 Y~%float64 Z~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'obstacle_map-response)))
  "Returns full string definition for message of type 'obstacle_map-response"
  (cl:format cl:nil "C11_Agent/C24C11_OSM  osm~%~%~%================================================================================~%MSG: C11_Agent/C24C11_OSM~%int32 TYPE~%int32 TYPE_STATIC  = 0~%int32 TYPE_DYNAMIC = 1~%int16 ID~%C11_Agent/coord[] COM~%C11_Agent/pathLocation[] EXTR_VIS~%C11_Agent/pathLocation[] EXTR_TOP~%C11_Agent/coord EXTR_3D~%================================================================================~%MSG: C11_Agent/coord~%float64 X~%float64 Y~%float64 Z~%================================================================================~%MSG: C11_Agent/pathLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <obstacle_map-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'osm))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <obstacle_map-response>))
  "Converts a ROS message object to a list"
  (cl:list 'obstacle_map-response
    (cl:cons ':osm (osm msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'obstacle_map)))
  'obstacle_map-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'obstacle_map)))
  'obstacle_map-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'obstacle_map)))
  "Returns string type for a service object of type '<obstacle_map>"
  "C11_Agent/obstacle_map")