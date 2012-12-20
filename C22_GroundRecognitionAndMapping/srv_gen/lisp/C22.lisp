; Auto-generated. Do not edit!


(cl:in-package C22_GroundRecognitionAndMapping-srv)


;//! \htmlinclude C22-request.msg.html

(cl:defclass <C22-request> (roslisp-msg-protocol:ros-message)
  ((azimuth_msg
    :reader azimuth_msg
    :initarg :azimuth_msg
    :type C22_GroundRecognitionAndMapping-msg:C0C22_AZI
    :initform (cl:make-instance 'C22_GroundRecognitionAndMapping-msg:C0C22_AZI))
   (camera_sample_rate_msg
    :reader camera_sample_rate_msg
    :initarg :camera_sample_rate_msg
    :type C22_GroundRecognitionAndMapping-msg:C0C22_CAM
    :initform (cl:make-instance 'C22_GroundRecognitionAndMapping-msg:C0C22_CAM))
   (laser_sample_rate_msg
    :reader laser_sample_rate_msg
    :initarg :laser_sample_rate_msg
    :type C22_GroundRecognitionAndMapping-msg:C0C22_LAZ
    :initform (cl:make-instance 'C22_GroundRecognitionAndMapping-msg:C0C22_LAZ))
   (safety_requirements
    :reader safety_requirements
    :initarg :safety_requirements
    :type C22_GroundRecognitionAndMapping-msg:C0C22_SAF
    :initform (cl:make-instance 'C22_GroundRecognitionAndMapping-msg:C0C22_SAF)))
)

(cl:defclass C22-request (<C22-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C22-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C22-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C22_GroundRecognitionAndMapping-srv:<C22-request> is deprecated: use C22_GroundRecognitionAndMapping-srv:C22-request instead.")))

(cl:ensure-generic-function 'azimuth_msg-val :lambda-list '(m))
(cl:defmethod azimuth_msg-val ((m <C22-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-srv:azimuth_msg-val is deprecated.  Use C22_GroundRecognitionAndMapping-srv:azimuth_msg instead.")
  (azimuth_msg m))

(cl:ensure-generic-function 'camera_sample_rate_msg-val :lambda-list '(m))
(cl:defmethod camera_sample_rate_msg-val ((m <C22-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-srv:camera_sample_rate_msg-val is deprecated.  Use C22_GroundRecognitionAndMapping-srv:camera_sample_rate_msg instead.")
  (camera_sample_rate_msg m))

(cl:ensure-generic-function 'laser_sample_rate_msg-val :lambda-list '(m))
(cl:defmethod laser_sample_rate_msg-val ((m <C22-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-srv:laser_sample_rate_msg-val is deprecated.  Use C22_GroundRecognitionAndMapping-srv:laser_sample_rate_msg instead.")
  (laser_sample_rate_msg m))

(cl:ensure-generic-function 'safety_requirements-val :lambda-list '(m))
(cl:defmethod safety_requirements-val ((m <C22-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-srv:safety_requirements-val is deprecated.  Use C22_GroundRecognitionAndMapping-srv:safety_requirements instead.")
  (safety_requirements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C22-request>) ostream)
  "Serializes a message object of type '<C22-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'azimuth_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'camera_sample_rate_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'laser_sample_rate_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'safety_requirements) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C22-request>) istream)
  "Deserializes a message object of type '<C22-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'azimuth_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'camera_sample_rate_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'laser_sample_rate_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'safety_requirements) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C22-request>)))
  "Returns string type for a service object of type '<C22-request>"
  "C22_GroundRecognitionAndMapping/C22Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C22-request)))
  "Returns string type for a service object of type 'C22-request"
  "C22_GroundRecognitionAndMapping/C22Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C22-request>)))
  "Returns md5sum for a message object of type '<C22-request>"
  "8c8585ab643e373df4d81a9496d25b57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C22-request)))
  "Returns md5sum for a message object of type 'C22-request"
  "8c8585ab643e373df4d81a9496d25b57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C22-request>)))
  "Returns full string definition for message of type '<C22-request>"
  (cl:format cl:nil "C22_GroundRecognitionAndMapping/C0C22_AZI azimuth_msg~%C22_GroundRecognitionAndMapping/C0C22_CAM camera_sample_rate_msg~%C22_GroundRecognitionAndMapping/C0C22_LAZ laser_sample_rate_msg~%C22_GroundRecognitionAndMapping/C0C22_SAF safety_requirements~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C0C22_AZI~%float32 azimuth~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C0C22_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C0C22_LAZ~%int32 sampleRatePerSec~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C0C22_SAF~%int32 safety_req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C22-request)))
  "Returns full string definition for message of type 'C22-request"
  (cl:format cl:nil "C22_GroundRecognitionAndMapping/C0C22_AZI azimuth_msg~%C22_GroundRecognitionAndMapping/C0C22_CAM camera_sample_rate_msg~%C22_GroundRecognitionAndMapping/C0C22_LAZ laser_sample_rate_msg~%C22_GroundRecognitionAndMapping/C0C22_SAF safety_requirements~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C0C22_AZI~%float32 azimuth~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C0C22_CAM~%int32 frameRatePerSec~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C0C22_LAZ~%int32 sampleRatePerSec~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C0C22_SAF~%int32 safety_req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C22-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'azimuth_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'camera_sample_rate_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'laser_sample_rate_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'safety_requirements))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C22-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C22-request
    (cl:cons ':azimuth_msg (azimuth_msg msg))
    (cl:cons ':camera_sample_rate_msg (camera_sample_rate_msg msg))
    (cl:cons ':laser_sample_rate_msg (laser_sample_rate_msg msg))
    (cl:cons ':safety_requirements (safety_requirements msg))
))
;//! \htmlinclude C22-response.msg.html

(cl:defclass <C22-response> (roslisp-msg-protocol:ros-message)
  ((drivingPath
    :reader drivingPath
    :initarg :drivingPath
    :type C22_GroundRecognitionAndMapping-msg:C22C0_PATH
    :initform (cl:make-instance 'C22_GroundRecognitionAndMapping-msg:C22C0_PATH)))
)

(cl:defclass C22-response (<C22-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C22-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C22-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C22_GroundRecognitionAndMapping-srv:<C22-response> is deprecated: use C22_GroundRecognitionAndMapping-srv:C22-response instead.")))

(cl:ensure-generic-function 'drivingPath-val :lambda-list '(m))
(cl:defmethod drivingPath-val ((m <C22-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-srv:drivingPath-val is deprecated.  Use C22_GroundRecognitionAndMapping-srv:drivingPath instead.")
  (drivingPath m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C22-response>) ostream)
  "Serializes a message object of type '<C22-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'drivingPath) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C22-response>) istream)
  "Deserializes a message object of type '<C22-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'drivingPath) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C22-response>)))
  "Returns string type for a service object of type '<C22-response>"
  "C22_GroundRecognitionAndMapping/C22Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C22-response)))
  "Returns string type for a service object of type 'C22-response"
  "C22_GroundRecognitionAndMapping/C22Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C22-response>)))
  "Returns md5sum for a message object of type '<C22-response>"
  "8c8585ab643e373df4d81a9496d25b57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C22-response)))
  "Returns md5sum for a message object of type 'C22-response"
  "8c8585ab643e373df4d81a9496d25b57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C22-response>)))
  "Returns full string definition for message of type '<C22-response>"
  (cl:format cl:nil "C22_GroundRecognitionAndMapping/C22C0_PATH drivingPath~%~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C22C0_PATH~%float32[] PATH_Left~%float32[] PATH_Right~%float32 PATH_Slope~%int32 PATH_ROU~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C22-response)))
  "Returns full string definition for message of type 'C22-response"
  (cl:format cl:nil "C22_GroundRecognitionAndMapping/C22C0_PATH drivingPath~%~%~%================================================================================~%MSG: C22_GroundRecognitionAndMapping/C22C0_PATH~%float32[] PATH_Left~%float32[] PATH_Right~%float32 PATH_Slope~%int32 PATH_ROU~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C22-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'drivingPath))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C22-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C22-response
    (cl:cons ':drivingPath (drivingPath msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C22)))
  'C22-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C22)))
  'C22-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C22)))
  "Returns string type for a service object of type '<C22>"
  "C22_GroundRecognitionAndMapping/C22")