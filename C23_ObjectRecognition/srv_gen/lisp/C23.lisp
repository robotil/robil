; Auto-generated. Do not edit!


(cl:in-package C23_ObjectRecognition-srv)


;//! \htmlinclude C23-request.msg.html

(cl:defclass <C23-request> (roslisp-msg-protocol:ros-message)
  ((search_mode
    :reader search_mode
    :initarg :search_mode
    :type C23_ObjectRecognition-msg:C0C23_SEC
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:C0C23_SEC))
   (search_area
    :reader search_area
    :initarg :search_area
    :type C23_ObjectRecognition-msg:C0C23_SAR
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:C0C23_SAR))
   (search_object
    :reader search_object
    :initarg :search_object
    :type C23_ObjectRecognition-msg:C0C23_SEOB
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:C0C23_SEOB)))
)

(cl:defclass C23-request (<C23-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C23-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C23-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-srv:<C23-request> is deprecated: use C23_ObjectRecognition-srv:C23-request instead.")))

(cl:ensure-generic-function 'search_mode-val :lambda-list '(m))
(cl:defmethod search_mode-val ((m <C23-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-srv:search_mode-val is deprecated.  Use C23_ObjectRecognition-srv:search_mode instead.")
  (search_mode m))

(cl:ensure-generic-function 'search_area-val :lambda-list '(m))
(cl:defmethod search_area-val ((m <C23-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-srv:search_area-val is deprecated.  Use C23_ObjectRecognition-srv:search_area instead.")
  (search_area m))

(cl:ensure-generic-function 'search_object-val :lambda-list '(m))
(cl:defmethod search_object-val ((m <C23-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-srv:search_object-val is deprecated.  Use C23_ObjectRecognition-srv:search_object instead.")
  (search_object m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C23-request>) ostream)
  "Serializes a message object of type '<C23-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'search_mode) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'search_area) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'search_object) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C23-request>) istream)
  "Deserializes a message object of type '<C23-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'search_mode) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'search_area) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'search_object) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C23-request>)))
  "Returns string type for a service object of type '<C23-request>"
  "C23_ObjectRecognition/C23Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23-request)))
  "Returns string type for a service object of type 'C23-request"
  "C23_ObjectRecognition/C23Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C23-request>)))
  "Returns md5sum for a message object of type '<C23-request>"
  "63a35c438ba90815eb22ce44ef5380f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C23-request)))
  "Returns md5sum for a message object of type 'C23-request"
  "63a35c438ba90815eb22ce44ef5380f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C23-request>)))
  "Returns full string definition for message of type '<C23-request>"
  (cl:format cl:nil "C23_ObjectRecognition/C0C23_SEC search_mode~%C23_ObjectRecognition/C0C23_SAR search_area~%C23_ObjectRecognition/C0C23_SEOB search_object~%~%================================================================================~%MSG: C23_ObjectRecognition/C0C23_SEC~%int32 searchMode~%~%================================================================================~%MSG: C23_ObjectRecognition/C0C23_SAR~%float32 SARV_L~%float32 SARV_U~%float32 SARH_L~%float32 SARH_R~%~%================================================================================~%MSG: C23_ObjectRecognition/C0C23_SEOB~%int32 search_object~%int32 WHEEL=0~%int32 DOOR=1~%int32 BREAK_PEDAL=2~%int32 STAIR=3~%int32 LADDER=4~%int32 VALVE=5~%int32 TOOL=6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C23-request)))
  "Returns full string definition for message of type 'C23-request"
  (cl:format cl:nil "C23_ObjectRecognition/C0C23_SEC search_mode~%C23_ObjectRecognition/C0C23_SAR search_area~%C23_ObjectRecognition/C0C23_SEOB search_object~%~%================================================================================~%MSG: C23_ObjectRecognition/C0C23_SEC~%int32 searchMode~%~%================================================================================~%MSG: C23_ObjectRecognition/C0C23_SAR~%float32 SARV_L~%float32 SARV_U~%float32 SARH_L~%float32 SARH_R~%~%================================================================================~%MSG: C23_ObjectRecognition/C0C23_SEOB~%int32 search_object~%int32 WHEEL=0~%int32 DOOR=1~%int32 BREAK_PEDAL=2~%int32 STAIR=3~%int32 LADDER=4~%int32 VALVE=5~%int32 TOOL=6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C23-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'search_mode))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'search_area))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'search_object))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C23-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C23-request
    (cl:cons ':search_mode (search_mode msg))
    (cl:cons ':search_area (search_area msg))
    (cl:cons ':search_object (search_object msg))
))
;//! \htmlinclude C23-response.msg.html

(cl:defclass <C23-response> (roslisp-msg-protocol:ros-message)
  ((objectDetected
    :reader objectDetected
    :initarg :objectDetected
    :type C23_ObjectRecognition-msg:C23C0_OD
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:C23C0_OD))
   (objectPossition
    :reader objectPossition
    :initarg :objectPossition
    :type C23_ObjectRecognition-msg:C23C0_OPO
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:C23C0_OPO))
   (objectDimensions
    :reader objectDimensions
    :initarg :objectDimensions
    :type C23_ObjectRecognition-msg:C23C0_ODIM
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:C23C0_ODIM)))
)

(cl:defclass C23-response (<C23-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C23-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C23-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-srv:<C23-response> is deprecated: use C23_ObjectRecognition-srv:C23-response instead.")))

(cl:ensure-generic-function 'objectDetected-val :lambda-list '(m))
(cl:defmethod objectDetected-val ((m <C23-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-srv:objectDetected-val is deprecated.  Use C23_ObjectRecognition-srv:objectDetected instead.")
  (objectDetected m))

(cl:ensure-generic-function 'objectPossition-val :lambda-list '(m))
(cl:defmethod objectPossition-val ((m <C23-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-srv:objectPossition-val is deprecated.  Use C23_ObjectRecognition-srv:objectPossition instead.")
  (objectPossition m))

(cl:ensure-generic-function 'objectDimensions-val :lambda-list '(m))
(cl:defmethod objectDimensions-val ((m <C23-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-srv:objectDimensions-val is deprecated.  Use C23_ObjectRecognition-srv:objectDimensions instead.")
  (objectDimensions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C23-response>) ostream)
  "Serializes a message object of type '<C23-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'objectDetected) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'objectPossition) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'objectDimensions) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C23-response>) istream)
  "Deserializes a message object of type '<C23-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'objectDetected) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'objectPossition) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'objectDimensions) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C23-response>)))
  "Returns string type for a service object of type '<C23-response>"
  "C23_ObjectRecognition/C23Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23-response)))
  "Returns string type for a service object of type 'C23-response"
  "C23_ObjectRecognition/C23Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C23-response>)))
  "Returns md5sum for a message object of type '<C23-response>"
  "63a35c438ba90815eb22ce44ef5380f8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C23-response)))
  "Returns md5sum for a message object of type 'C23-response"
  "63a35c438ba90815eb22ce44ef5380f8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C23-response>)))
  "Returns full string definition for message of type '<C23-response>"
  (cl:format cl:nil "C23_ObjectRecognition/C23C0_OD objectDetected~%C23_ObjectRecognition/C23C0_OPO objectPossition~%C23_ObjectRecognition/C23C0_ODIM objectDimensions~%~%~%================================================================================~%MSG: C23_ObjectRecognition/C23C0_OD~%int32 ObjectDetected~%~%================================================================================~%MSG: C23_ObjectRecognition/C23C0_OPO~%C23_ObjectRecognition/TBD position~%~%================================================================================~%MSG: C23_ObjectRecognition/TBD~%int32 x~%int32 y~%int32 z~%~%================================================================================~%MSG: C23_ObjectRecognition/C23C0_ODIM~%C23_ObjectRecognition/TBD min_dimensions~%C23_ObjectRecognition/TBD max_dimensions~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C23-response)))
  "Returns full string definition for message of type 'C23-response"
  (cl:format cl:nil "C23_ObjectRecognition/C23C0_OD objectDetected~%C23_ObjectRecognition/C23C0_OPO objectPossition~%C23_ObjectRecognition/C23C0_ODIM objectDimensions~%~%~%================================================================================~%MSG: C23_ObjectRecognition/C23C0_OD~%int32 ObjectDetected~%~%================================================================================~%MSG: C23_ObjectRecognition/C23C0_OPO~%C23_ObjectRecognition/TBD position~%~%================================================================================~%MSG: C23_ObjectRecognition/TBD~%int32 x~%int32 y~%int32 z~%~%================================================================================~%MSG: C23_ObjectRecognition/C23C0_ODIM~%C23_ObjectRecognition/TBD min_dimensions~%C23_ObjectRecognition/TBD max_dimensions~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C23-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'objectDetected))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'objectPossition))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'objectDimensions))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C23-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C23-response
    (cl:cons ':objectDetected (objectDetected msg))
    (cl:cons ':objectPossition (objectPossition msg))
    (cl:cons ':objectDimensions (objectDimensions msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C23)))
  'C23-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C23)))
  'C23-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23)))
  "Returns string type for a service object of type '<C23>"
  "C23_ObjectRecognition/C23")