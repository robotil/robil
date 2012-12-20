; Auto-generated. Do not edit!


(cl:in-package C25_GlobalPosition-srv)


;//! \htmlinclude C25-request.msg.html

(cl:defclass <C25-request> (roslisp-msg-protocol:ros-message)
  ((azimuth
    :reader azimuth
    :initarg :azimuth
    :type C25_GlobalPosition-msg:C0C25_AZI
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C0C25_AZI))
   (camera_sample_rate
    :reader camera_sample_rate
    :initarg :camera_sample_rate
    :type C25_GlobalPosition-msg:C0C25_CAM
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C0C25_CAM))
   (laser_sample_rate
    :reader laser_sample_rate
    :initarg :laser_sample_rate
    :type C25_GlobalPosition-msg:C0C25_LAZ
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C0C25_LAZ)))
)

(cl:defclass C25-request (<C25-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C25-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C25-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C25_GlobalPosition-srv:<C25-request> is deprecated: use C25_GlobalPosition-srv:C25-request instead.")))

(cl:ensure-generic-function 'azimuth-val :lambda-list '(m))
(cl:defmethod azimuth-val ((m <C25-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:azimuth-val is deprecated.  Use C25_GlobalPosition-srv:azimuth instead.")
  (azimuth m))

(cl:ensure-generic-function 'camera_sample_rate-val :lambda-list '(m))
(cl:defmethod camera_sample_rate-val ((m <C25-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:camera_sample_rate-val is deprecated.  Use C25_GlobalPosition-srv:camera_sample_rate instead.")
  (camera_sample_rate m))

(cl:ensure-generic-function 'laser_sample_rate-val :lambda-list '(m))
(cl:defmethod laser_sample_rate-val ((m <C25-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:laser_sample_rate-val is deprecated.  Use C25_GlobalPosition-srv:laser_sample_rate instead.")
  (laser_sample_rate m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C25-request>) ostream)
  "Serializes a message object of type '<C25-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'azimuth) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_sample_rate) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'laser_sample_rate) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C25-request>) istream)
  "Deserializes a message object of type '<C25-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'azimuth) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_sample_rate) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'laser_sample_rate) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C25-request>)))
  "Returns string type for a service object of type '<C25-request>"
  "C25_GlobalPosition/C25Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C25-request)))
  "Returns string type for a service object of type 'C25-request"
  "C25_GlobalPosition/C25Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C25-request>)))
  "Returns md5sum for a message object of type '<C25-request>"
  "f2e36a28014593f1ba0d9b48126bdf08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C25-request)))
  "Returns md5sum for a message object of type 'C25-request"
  "f2e36a28014593f1ba0d9b48126bdf08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C25-request>)))
  "Returns full string definition for message of type '<C25-request>"
  (cl:format cl:nil "C25_GlobalPosition/C0C25_AZI azimuth~%C25_GlobalPosition/C0C25_CAM camera_sample_rate~%C25_GlobalPosition/C0C25_LAZ laser_sample_rate~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_AZI~%float32 azimuth~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_LAZ~%int32 sampleRatePerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C25-request)))
  "Returns full string definition for message of type 'C25-request"
  (cl:format cl:nil "C25_GlobalPosition/C0C25_AZI azimuth~%C25_GlobalPosition/C0C25_CAM camera_sample_rate~%C25_GlobalPosition/C0C25_LAZ laser_sample_rate~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_AZI~%float32 azimuth~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C25_GlobalPosition/C0C25_LAZ~%int32 sampleRatePerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C25-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'azimuth))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_sample_rate))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'laser_sample_rate))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C25-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C25-request
    (cl:cons ':azimuth (azimuth msg))
    (cl:cons ':camera_sample_rate (camera_sample_rate msg))
    (cl:cons ':laser_sample_rate (laser_sample_rate msg))
))
;//! \htmlinclude C25-response.msg.html

(cl:defclass <C25-response> (roslisp-msg-protocol:ros-message)
  ((robotPosition
    :reader robotPosition
    :initarg :robotPosition
    :type C25_GlobalPosition-msg:C25C0_ROP
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C25C0_ROP))
   (qualityOfError
    :reader qualityOfError
    :initarg :qualityOfError
    :type C25_GlobalPosition-msg:C25C0_OPO
    :initform (cl:make-instance 'C25_GlobalPosition-msg:C25C0_OPO)))
)

(cl:defclass C25-response (<C25-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C25-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C25-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C25_GlobalPosition-srv:<C25-response> is deprecated: use C25_GlobalPosition-srv:C25-response instead.")))

(cl:ensure-generic-function 'robotPosition-val :lambda-list '(m))
(cl:defmethod robotPosition-val ((m <C25-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:robotPosition-val is deprecated.  Use C25_GlobalPosition-srv:robotPosition instead.")
  (robotPosition m))

(cl:ensure-generic-function 'qualityOfError-val :lambda-list '(m))
(cl:defmethod qualityOfError-val ((m <C25-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-srv:qualityOfError-val is deprecated.  Use C25_GlobalPosition-srv:qualityOfError instead.")
  (qualityOfError m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C25-response>) ostream)
  "Serializes a message object of type '<C25-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'robotPosition) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'qualityOfError) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C25-response>) istream)
  "Deserializes a message object of type '<C25-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'robotPosition) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'qualityOfError) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C25-response>)))
  "Returns string type for a service object of type '<C25-response>"
  "C25_GlobalPosition/C25Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C25-response)))
  "Returns string type for a service object of type 'C25-response"
  "C25_GlobalPosition/C25Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C25-response>)))
  "Returns md5sum for a message object of type '<C25-response>"
  "f2e36a28014593f1ba0d9b48126bdf08")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C25-response)))
  "Returns md5sum for a message object of type 'C25-response"
  "f2e36a28014593f1ba0d9b48126bdf08")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C25-response>)))
  "Returns full string definition for message of type '<C25-response>"
  (cl:format cl:nil "C25_GlobalPosition/C25C0_ROP robotPosition~%C25_GlobalPosition/C25C0_OPO qualityOfError~%~%~%================================================================================~%MSG: C25_GlobalPosition/C25C0_ROP~%C25_GlobalPosition/UTM min_dimensions~%~%~%================================================================================~%MSG: C25_GlobalPosition/UTM~%int32 zone~%int64 easting~%int64 northing~%int32 hemisphere~%int32 NORTHERN_HEMISPHERE=0~%int32 SOUTHERN_HEMISPHERE=1~%~%================================================================================~%MSG: C25_GlobalPosition/C25C0_OPO~%float32 qualityOfPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C25-response)))
  "Returns full string definition for message of type 'C25-response"
  (cl:format cl:nil "C25_GlobalPosition/C25C0_ROP robotPosition~%C25_GlobalPosition/C25C0_OPO qualityOfError~%~%~%================================================================================~%MSG: C25_GlobalPosition/C25C0_ROP~%C25_GlobalPosition/UTM min_dimensions~%~%~%================================================================================~%MSG: C25_GlobalPosition/UTM~%int32 zone~%int64 easting~%int64 northing~%int32 hemisphere~%int32 NORTHERN_HEMISPHERE=0~%int32 SOUTHERN_HEMISPHERE=1~%~%================================================================================~%MSG: C25_GlobalPosition/C25C0_OPO~%float32 qualityOfPosition~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C25-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'robotPosition))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'qualityOfError))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C25-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C25-response
    (cl:cons ':robotPosition (robotPosition msg))
    (cl:cons ':qualityOfError (qualityOfError msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C25)))
  'C25-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C25)))
  'C25-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C25)))
  "Returns string type for a service object of type '<C25>"
  "C25_GlobalPosition/C25")