; Auto-generated. Do not edit!


(cl:in-package leg_ik-srv)


;//! \htmlinclude LegIk-request.msg.html

(cl:defclass <LegIk-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type leg_ik-msg:traj
    :initform (cl:make-instance 'leg_ik-msg:traj)))
)

(cl:defclass LegIk-request (<LegIk-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegIk-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegIk-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leg_ik-srv:<LegIk-request> is deprecated: use leg_ik-srv:LegIk-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <LegIk-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-srv:pos-val is deprecated.  Use leg_ik-srv:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegIk-request>) ostream)
  "Serializes a message object of type '<LegIk-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegIk-request>) istream)
  "Deserializes a message object of type '<LegIk-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegIk-request>)))
  "Returns string type for a service object of type '<LegIk-request>"
  "leg_ik/LegIkRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegIk-request)))
  "Returns string type for a service object of type 'LegIk-request"
  "leg_ik/LegIkRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegIk-request>)))
  "Returns md5sum for a message object of type '<LegIk-request>"
  "b11527b1e6596cdbc96c7b94f75be9c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegIk-request)))
  "Returns md5sum for a message object of type 'LegIk-request"
  "b11527b1e6596cdbc96c7b94f75be9c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegIk-request>)))
  "Returns full string definition for message of type '<LegIk-request>"
  (cl:format cl:nil "leg_ik/traj pos~%~%================================================================================~%MSG: leg_ik/traj~%float64 COMx~%float64 COMy~%float64 COMz~%float64 Swing_x~%float64 Swing_y~%float64 Swing_z~%int32   leg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegIk-request)))
  "Returns full string definition for message of type 'LegIk-request"
  (cl:format cl:nil "leg_ik/traj pos~%~%================================================================================~%MSG: leg_ik/traj~%float64 COMx~%float64 COMy~%float64 COMz~%float64 Swing_x~%float64 Swing_y~%float64 Swing_z~%int32   leg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegIk-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegIk-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LegIk-request
    (cl:cons ':pos (pos msg))
))
;//! \htmlinclude LegIk-response.msg.html

(cl:defclass <LegIk-response> (roslisp-msg-protocol:ros-message)
  ((ang
    :reader ang
    :initarg :ang
    :type leg_ik-msg:LegAngle
    :initform (cl:make-instance 'leg_ik-msg:LegAngle)))
)

(cl:defclass LegIk-response (<LegIk-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegIk-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegIk-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leg_ik-srv:<LegIk-response> is deprecated: use leg_ik-srv:LegIk-response instead.")))

(cl:ensure-generic-function 'ang-val :lambda-list '(m))
(cl:defmethod ang-val ((m <LegIk-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-srv:ang-val is deprecated.  Use leg_ik-srv:ang instead.")
  (ang m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegIk-response>) ostream)
  "Serializes a message object of type '<LegIk-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ang) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegIk-response>) istream)
  "Deserializes a message object of type '<LegIk-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ang) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegIk-response>)))
  "Returns string type for a service object of type '<LegIk-response>"
  "leg_ik/LegIkResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegIk-response)))
  "Returns string type for a service object of type 'LegIk-response"
  "leg_ik/LegIkResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegIk-response>)))
  "Returns md5sum for a message object of type '<LegIk-response>"
  "b11527b1e6596cdbc96c7b94f75be9c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegIk-response)))
  "Returns md5sum for a message object of type 'LegIk-response"
  "b11527b1e6596cdbc96c7b94f75be9c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegIk-response>)))
  "Returns full string definition for message of type '<LegIk-response>"
  (cl:format cl:nil "leg_ik/LegAngle ang~%~%================================================================================~%MSG: leg_ik/LegAngle~%float64 mhx~%float64 lhy~%float64 uhz~%float64 kny~%float64 lax~%float64 uay~%float64 mby~%float64 ubx~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegIk-response)))
  "Returns full string definition for message of type 'LegIk-response"
  (cl:format cl:nil "leg_ik/LegAngle ang~%~%================================================================================~%MSG: leg_ik/LegAngle~%float64 mhx~%float64 lhy~%float64 uhz~%float64 kny~%float64 lax~%float64 uay~%float64 mby~%float64 ubx~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegIk-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ang))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegIk-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LegIk-response
    (cl:cons ':ang (ang msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LegIk)))
  'LegIk-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LegIk)))
  'LegIk-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegIk)))
  "Returns string type for a service object of type '<LegIk>"
  "leg_ik/LegIk")