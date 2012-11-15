; Auto-generated. Do not edit!


(cl:in-package C24_ObstacleDetection-srv)


;//! \htmlinclude C24-request.msg.html

(cl:defclass <C24-request> (roslisp-msg-protocol:ros-message)
  ((azimuth
    :reader azimuth
    :initarg :azimuth
    :type C24_ObstacleDetection-msg:C0C24_AZI
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:C0C24_AZI))
   (camera_sample_rate
    :reader camera_sample_rate
    :initarg :camera_sample_rate
    :type C24_ObstacleDetection-msg:C0C24_CAM
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:C0C24_CAM))
   (laser_sample_rate
    :reader laser_sample_rate
    :initarg :laser_sample_rate
    :type C24_ObstacleDetection-msg:C0C24_LAZ
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:C0C24_LAZ))
   (minimum_obstacle_size
    :reader minimum_obstacle_size
    :initarg :minimum_obstacle_size
    :type C24_ObstacleDetection-msg:C0C24_SIZ
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:C0C24_SIZ)))
)

(cl:defclass C24-request (<C24-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C24-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C24-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C24_ObstacleDetection-srv:<C24-request> is deprecated: use C24_ObstacleDetection-srv:C24-request instead.")))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <C24-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-srv:azimuth-val is deprecated.  Use C24_ObstacleDetection-srv:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'camera_sample_rate-val :lambda-list '(m))
(cl:defmethod camera_sample_rate-val ((m <C24-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-srv:camera_sample_rate-val is deprecated.  Use C24_ObstacleDetection-srv:camera_sample_rate instead.")
  (camera_sample_rate m))

(cl:ensure-generic-function 'laser_sample_rate-val :lambda-list '(m))
(cl:defmethod laser_sample_rate-val ((m <C24-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-srv:laser_sample_rate-val is deprecated.  Use C24_ObstacleDetection-srv:laser_sample_rate instead.")
  (laser_sample_rate m))

(cl:ensure-generic-function 'minimum_obstacle_size-val :lambda-list '(m))
(cl:defmethod minimum_obstacle_size-val ((m <C24-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-srv:minimum_obstacle_size-val is deprecated.  Use C24_ObstacleDetection-srv:minimum_obstacle_size instead.")
  (minimum_obstacle_size m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C24-request>) ostream)
  "Serializes a message object of type '<C24-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'azimuth) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_sample_rate) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'laser_sample_rate) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'minimum_obstacle_size) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C24-request>) istream)
  "Deserializes a message object of type '<C24-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'azimuth) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_sample_rate) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'laser_sample_rate) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'minimum_obstacle_size) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C24-request>)))
  "Returns string type for a service object of type '<C24-request>"
  "C24_ObstacleDetection/C24Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C24-request)))
  "Returns string type for a service object of type 'C24-request"
  "C24_ObstacleDetection/C24Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C24-request>)))
  "Returns md5sum for a message object of type '<C24-request>"
  "8f0e85e5c10133ee7ef1795738e66441")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C24-request)))
  "Returns md5sum for a message object of type 'C24-request"
  "8f0e85e5c10133ee7ef1795738e66441")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C24-request>)))
  "Returns full string definition for message of type '<C24-request>"
  (cl:format cl:nil "C24_ObstacleDetection/C0C24_AZI azimuth~%C24_ObstacleDetection/C0C24_CAM camera_sample_rate~%C24_ObstacleDetection/C0C24_LAZ laser_sample_rate~%C24_ObstacleDetection/C0C24_SIZ minimum_obstacle_size~%~%================================================================================~%MSG: C24_ObstacleDetection/C0C24_AZI~%float32 azimuth~%~%================================================================================~%MSG: C24_ObstacleDetection/C0C24_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C24_ObstacleDetection/C0C24_LAZ~%int32 sampleRatePerSec~%~%================================================================================~%MSG: C24_ObstacleDetection/C0C24_SIZ~%int32 image_width~%int32 image_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C24-request)))
  "Returns full string definition for message of type 'C24-request"
  (cl:format cl:nil "C24_ObstacleDetection/C0C24_AZI azimuth~%C24_ObstacleDetection/C0C24_CAM camera_sample_rate~%C24_ObstacleDetection/C0C24_LAZ laser_sample_rate~%C24_ObstacleDetection/C0C24_SIZ minimum_obstacle_size~%~%================================================================================~%MSG: C24_ObstacleDetection/C0C24_AZI~%float32 azimuth~%~%================================================================================~%MSG: C24_ObstacleDetection/C0C24_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C24_ObstacleDetection/C0C24_LAZ~%int32 sampleRatePerSec~%~%================================================================================~%MSG: C24_ObstacleDetection/C0C24_SIZ~%int32 image_width~%int32 image_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C24-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'azimuth))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_sample_rate))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'laser_sample_rate))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'minimum_obstacle_size))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C24-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C24-request
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':camera_sample_rate (camera_sample_rate msg))
    (cl:cons ':laser_sample_rate (laser_sample_rate msg))
    (cl:cons ':minimum_obstacle_size (minimum_obstacle_size msg))
))
;//! \htmlinclude C24-response.msg.html

(cl:defclass <C24-response> (roslisp-msg-protocol:ros-message)
  ((objectDetected
    :reader objectDetected
    :initarg :objectDetected
    :type C24_ObstacleDetection-msg:C24C0_OD
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:C24C0_OD))
   (objectPossition
    :reader objectPossition
    :initarg :objectPossition
    :type C24_ObstacleDetection-msg:C24C0_OPO
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:C24C0_OPO))
   (objectDimensions
    :reader objectDimensions
    :initarg :objectDimensions
    :type C24_ObstacleDetection-msg:C24C0_ODIM
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:C24C0_ODIM)))
)

(cl:defclass C24-response (<C24-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C24-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C24-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C24_ObstacleDetection-srv:<C24-response> is deprecated: use C24_ObstacleDetection-srv:C24-response instead.")))

(cl:ensure-generic-function 'objectDetected-val :lambda-list '(m))
(cl:defmethod objectDetected-val ((m <C24-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-srv:objectDetected-val is deprecated.  Use C24_ObstacleDetection-srv:objectDetected instead.")
  (objectDetected m))

(cl:ensure-generic-function 'objectPossition-val :lambda-list '(m))
(cl:defmethod objectPossition-val ((m <C24-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-srv:objectPossition-val is deprecated.  Use C24_ObstacleDetection-srv:objectPossition instead.")
  (objectPossition m))

(cl:ensure-generic-function 'objectDimensions-val :lambda-list '(m))
(cl:defmethod objectDimensions-val ((m <C24-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-srv:objectDimensions-val is deprecated.  Use C24_ObstacleDetection-srv:objectDimensions instead.")
  (objectDimensions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C24-response>) ostream)
  "Serializes a message object of type '<C24-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'objectDetected) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'objectPossition) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'objectDimensions) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C24-response>) istream)
  "Deserializes a message object of type '<C24-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'objectDetected) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'objectPossition) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'objectDimensions) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C24-response>)))
  "Returns string type for a service object of type '<C24-response>"
  "C24_ObstacleDetection/C24Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C24-response)))
  "Returns string type for a service object of type 'C24-response"
  "C24_ObstacleDetection/C24Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C24-response>)))
  "Returns md5sum for a message object of type '<C24-response>"
  "8f0e85e5c10133ee7ef1795738e66441")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C24-response)))
  "Returns md5sum for a message object of type 'C24-response"
  "8f0e85e5c10133ee7ef1795738e66441")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C24-response>)))
  "Returns full string definition for message of type '<C24-response>"
  (cl:format cl:nil "C24_ObstacleDetection/C24C0_OD objectDetected~%C24_ObstacleDetection/C24C0_OPO objectPossition~%C24_ObstacleDetection/C24C0_ODIM objectDimensions~%~%~%================================================================================~%MSG: C24_ObstacleDetection/C24C0_OD~%int32 ObjectDetected~%~%================================================================================~%MSG: C24_ObstacleDetection/C24C0_OPO~%C24_ObstacleDetection/TBD position~%~%================================================================================~%MSG: C24_ObstacleDetection/TBD~%int32 x~%int32 y~%int32 z~%~%================================================================================~%MSG: C24_ObstacleDetection/C24C0_ODIM~%C24_ObstacleDetection/TBD min_dimensions~%C24_ObstacleDetection/TBD max_dimensions~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C24-response)))
  "Returns full string definition for message of type 'C24-response"
  (cl:format cl:nil "C24_ObstacleDetection/C24C0_OD objectDetected~%C24_ObstacleDetection/C24C0_OPO objectPossition~%C24_ObstacleDetection/C24C0_ODIM objectDimensions~%~%~%================================================================================~%MSG: C24_ObstacleDetection/C24C0_OD~%int32 ObjectDetected~%~%================================================================================~%MSG: C24_ObstacleDetection/C24C0_OPO~%C24_ObstacleDetection/TBD position~%~%================================================================================~%MSG: C24_ObstacleDetection/TBD~%int32 x~%int32 y~%int32 z~%~%================================================================================~%MSG: C24_ObstacleDetection/C24C0_ODIM~%C24_ObstacleDetection/TBD min_dimensions~%C24_ObstacleDetection/TBD max_dimensions~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C24-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'objectDetected))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'objectPossition))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'objectDimensions))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C24-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C24-response
    (cl:cons ':objectDetected (objectDetected msg))
    (cl:cons ':objectPossition (objectPossition msg))
    (cl:cons ':objectDimensions (objectDimensions msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C24)))
  'C24-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C24)))
  'C24-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C24)))
  "Returns string type for a service object of type '<C24>"
  "C24_ObstacleDetection/C24")