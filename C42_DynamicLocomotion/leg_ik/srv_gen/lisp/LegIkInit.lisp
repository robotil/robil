; Auto-generated. Do not edit!


(cl:in-package leg_ik-srv)


;//! \htmlinclude LegIkInit-request.msg.html

(cl:defclass <LegIkInit-request> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type leg_ik-msg:traj
    :initform (cl:make-instance 'leg_ik-msg:traj)))
)

(cl:defclass LegIkInit-request (<LegIkInit-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegIkInit-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegIkInit-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leg_ik-srv:<LegIkInit-request> is deprecated: use leg_ik-srv:LegIkInit-request instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <LegIkInit-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-srv:pos-val is deprecated.  Use leg_ik-srv:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegIkInit-request>) ostream)
  "Serializes a message object of type '<LegIkInit-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'pos) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegIkInit-request>) istream)
  "Deserializes a message object of type '<LegIkInit-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'pos) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegIkInit-request>)))
  "Returns string type for a service object of type '<LegIkInit-request>"
  "leg_ik/LegIkInitRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegIkInit-request)))
  "Returns string type for a service object of type 'LegIkInit-request"
  "leg_ik/LegIkInitRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegIkInit-request>)))
  "Returns md5sum for a message object of type '<LegIkInit-request>"
  "b11527b1e6596cdbc96c7b94f75be9c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegIkInit-request)))
  "Returns md5sum for a message object of type 'LegIkInit-request"
  "b11527b1e6596cdbc96c7b94f75be9c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegIkInit-request>)))
  "Returns full string definition for message of type '<LegIkInit-request>"
  (cl:format cl:nil "leg_ik/traj pos~%~%================================================================================~%MSG: leg_ik/traj~%float64 COMx~%float64 COMy~%float64 COMz~%float64 Swing_x~%float64 Swing_y~%float64 Swing_z~%int32   leg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegIkInit-request)))
  "Returns full string definition for message of type 'LegIkInit-request"
  (cl:format cl:nil "leg_ik/traj pos~%~%================================================================================~%MSG: leg_ik/traj~%float64 COMx~%float64 COMy~%float64 COMz~%float64 Swing_x~%float64 Swing_y~%float64 Swing_z~%int32   leg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegIkInit-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'pos))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegIkInit-request>))
  "Converts a ROS message object to a list"
  (cl:list 'LegIkInit-request
    (cl:cons ':pos (pos msg))
))
;//! \htmlinclude LegIkInit-response.msg.html

(cl:defclass <LegIkInit-response> (roslisp-msg-protocol:ros-message)
  ((ang
    :reader ang
    :initarg :ang
    :type leg_ik-msg:LegAngle
    :initform (cl:make-instance 'leg_ik-msg:LegAngle)))
)

(cl:defclass LegIkInit-response (<LegIkInit-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <LegIkInit-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'LegIkInit-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leg_ik-srv:<LegIkInit-response> is deprecated: use leg_ik-srv:LegIkInit-response instead.")))

(cl:ensure-generic-function 'ang-val :lambda-list '(m))
(cl:defmethod ang-val ((m <LegIkInit-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-srv:ang-val is deprecated.  Use leg_ik-srv:ang instead.")
  (ang m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <LegIkInit-response>) ostream)
  "Serializes a message object of type '<LegIkInit-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'ang) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <LegIkInit-response>) istream)
  "Deserializes a message object of type '<LegIkInit-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'ang) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<LegIkInit-response>)))
  "Returns string type for a service object of type '<LegIkInit-response>"
  "leg_ik/LegIkInitResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegIkInit-response)))
  "Returns string type for a service object of type 'LegIkInit-response"
  "leg_ik/LegIkInitResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<LegIkInit-response>)))
  "Returns md5sum for a message object of type '<LegIkInit-response>"
  "b11527b1e6596cdbc96c7b94f75be9c7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'LegIkInit-response)))
  "Returns md5sum for a message object of type 'LegIkInit-response"
  "b11527b1e6596cdbc96c7b94f75be9c7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<LegIkInit-response>)))
  "Returns full string definition for message of type '<LegIkInit-response>"
  (cl:format cl:nil "leg_ik/LegAngle ang~%~%~%================================================================================~%MSG: leg_ik/LegAngle~%float64 mhx~%float64 lhy~%float64 uhz~%float64 kny~%float64 lax~%float64 uay~%float64 mby~%float64 ubx~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'LegIkInit-response)))
  "Returns full string definition for message of type 'LegIkInit-response"
  (cl:format cl:nil "leg_ik/LegAngle ang~%~%~%================================================================================~%MSG: leg_ik/LegAngle~%float64 mhx~%float64 lhy~%float64 uhz~%float64 kny~%float64 lax~%float64 uay~%float64 mby~%float64 ubx~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <LegIkInit-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'ang))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <LegIkInit-response>))
  "Converts a ROS message object to a list"
  (cl:list 'LegIkInit-response
    (cl:cons ':ang (ang msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'LegIkInit)))
  'LegIkInit-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'LegIkInit)))
  'LegIkInit-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'LegIkInit)))
  "Returns string type for a service object of type '<LegIkInit>"
  "leg_ik/LegIkInit")