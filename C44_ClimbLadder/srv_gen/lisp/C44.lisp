; Auto-generated. Do not edit!


(cl:in-package C44_ClimbLadder-srv)


;//! \htmlinclude C44-request.msg.html

(cl:defclass <C44-request> (roslisp-msg-protocol:ros-message)
  ((ladder_dimensions_msg
    :reader ladder_dimensions_msg
    :initarg :ladder_dimensions_msg
    :type C44_ClimbLadder-msg:C23C44_LDIM
    :initform (cl:make-instance 'C44_ClimbLadder-msg:C23C44_LDIM))
   (distance_between_steps_msg
    :reader distance_between_steps_msg
    :initarg :distance_between_steps_msg
    :type C44_ClimbLadder-msg:C23C44_DIS
    :initform (cl:make-instance 'C44_ClimbLadder-msg:C23C44_DIS)))
)

(cl:defclass C44-request (<C44-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C44-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C44-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C44_ClimbLadder-srv:<C44-request> is deprecated: use C44_ClimbLadder-srv:C44-request instead.")))

(cl:ensure-generic-function 'ladder_dimensions_msg-val :lambda-list '(m))
(cl:defmethod ladder_dimensions_msg-val ((m <C44-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C44_ClimbLadder-srv:ladder_dimensions_msg-val is deprecated.  Use C44_ClimbLadder-srv:ladder_dimensions_msg instead.")
  (ladder_dimensions_msg m))

(cl:ensure-generic-function 'distance_between_steps_msg-val :lambda-list '(m))
(cl:defmethod distance_between_steps_msg-val ((m <C44-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C44_ClimbLadder-srv:distance_between_steps_msg-val is deprecated.  Use C44_ClimbLadder-srv:distance_between_steps_msg instead.")
  (distance_between_steps_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C44-request>) ostream)
  "Serializes a message object of type '<C44-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ladder_dimensions_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'distance_between_steps_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C44-request>) istream)
  "Deserializes a message object of type '<C44-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ladder_dimensions_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'distance_between_steps_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C44-request>)))
  "Returns string type for a service object of type '<C44-request>"
  "C44_ClimbLadder/C44Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C44-request)))
  "Returns string type for a service object of type 'C44-request"
  "C44_ClimbLadder/C44Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C44-request>)))
  "Returns md5sum for a message object of type '<C44-request>"
  "a70e7b52412f77b8f72fb49133206e55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C44-request)))
  "Returns md5sum for a message object of type 'C44-request"
  "a70e7b52412f77b8f72fb49133206e55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C44-request>)))
  "Returns full string definition for message of type '<C44-request>"
  (cl:format cl:nil "C44_ClimbLadder/C23C44_LDIM ladder_dimensions_msg~%C44_ClimbLadder/C23C44_DIS distance_between_steps_msg~%~%================================================================================~%MSG: C44_ClimbLadder/C23C44_LDIM~%float32[] ladder_dimensions~%~%================================================================================~%MSG: C44_ClimbLadder/C23C44_DIS~%float32 distance_between_steps~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C44-request)))
  "Returns full string definition for message of type 'C44-request"
  (cl:format cl:nil "C44_ClimbLadder/C23C44_LDIM ladder_dimensions_msg~%C44_ClimbLadder/C23C44_DIS distance_between_steps_msg~%~%================================================================================~%MSG: C44_ClimbLadder/C23C44_LDIM~%float32[] ladder_dimensions~%~%================================================================================~%MSG: C44_ClimbLadder/C23C44_DIS~%float32 distance_between_steps~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C44-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ladder_dimensions_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'distance_between_steps_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C44-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C44-request
    (cl:cons ':ladder_dimensions_msg (ladder_dimensions_msg msg))
    (cl:cons ':distance_between_steps_msg (distance_between_steps_msg msg))
))
;//! \htmlinclude C44-response.msg.html

(cl:defclass <C44-response> (roslisp-msg-protocol:ros-message)
  ((climb_a_ladder_Success_msg
    :reader climb_a_ladder_Success_msg
    :initarg :climb_a_ladder_Success_msg
    :type C44_ClimbLadder-msg:C44C0_CLS
    :initform (cl:make-instance 'C44_ClimbLadder-msg:C44C0_CLS)))
)

(cl:defclass C44-response (<C44-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C44-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C44-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C44_ClimbLadder-srv:<C44-response> is deprecated: use C44_ClimbLadder-srv:C44-response instead.")))

(cl:ensure-generic-function 'climb_a_ladder_Success_msg-val :lambda-list '(m))
(cl:defmethod climb_a_ladder_Success_msg-val ((m <C44-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C44_ClimbLadder-srv:climb_a_ladder_Success_msg-val is deprecated.  Use C44_ClimbLadder-srv:climb_a_ladder_Success_msg instead.")
  (climb_a_ladder_Success_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C44-response>) ostream)
  "Serializes a message object of type '<C44-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'climb_a_ladder_Success_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C44-response>) istream)
  "Deserializes a message object of type '<C44-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'climb_a_ladder_Success_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C44-response>)))
  "Returns string type for a service object of type '<C44-response>"
  "C44_ClimbLadder/C44Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C44-response)))
  "Returns string type for a service object of type 'C44-response"
  "C44_ClimbLadder/C44Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C44-response>)))
  "Returns md5sum for a message object of type '<C44-response>"
  "a70e7b52412f77b8f72fb49133206e55")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C44-response)))
  "Returns md5sum for a message object of type 'C44-response"
  "a70e7b52412f77b8f72fb49133206e55")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C44-response>)))
  "Returns full string definition for message of type '<C44-response>"
  (cl:format cl:nil "C44_ClimbLadder/C44C0_CLS climb_a_ladder_Success_msg~%~%~%~%================================================================================~%MSG: C44_ClimbLadder/C44C0_CLS~%bool climb_a_ladder_success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C44-response)))
  "Returns full string definition for message of type 'C44-response"
  (cl:format cl:nil "C44_ClimbLadder/C44C0_CLS climb_a_ladder_Success_msg~%~%~%~%================================================================================~%MSG: C44_ClimbLadder/C44C0_CLS~%bool climb_a_ladder_success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C44-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'climb_a_ladder_Success_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C44-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C44-response
    (cl:cons ':climb_a_ladder_Success_msg (climb_a_ladder_Success_msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C44)))
  'C44-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C44)))
  'C44-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C44)))
  "Returns string type for a service object of type '<C44>"
  "C44_ClimbLadder/C44")