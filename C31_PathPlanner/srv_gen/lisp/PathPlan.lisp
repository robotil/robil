; Auto-generated. Do not edit!


(cl:in-package C31_PathPlanner-srv)


;//! \htmlinclude PathPlan-request.msg.html

(cl:defclass <PathPlan-request> (roslisp-msg-protocol:ros-message)
  ((map
    :reader map
    :initarg :map
    :type C31_PathPlanner-msg:ppMap
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppMap))
   (constraints
    :reader constraints
    :initarg :constraints
    :type C31_PathPlanner-msg:ppConstraints
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppConstraints))
   (start
    :reader start
    :initarg :start
    :type C31_PathPlanner-msg:ppLocation
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppLocation))
   (destination
    :reader destination
    :initarg :destination
    :type C31_PathPlanner-msg:ppLocation
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppLocation))
   (path
    :reader path
    :initarg :path
    :type C31_PathPlanner-msg:ppWaypoints
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppWaypoints))
   (algorith
    :reader algorith
    :initarg :algorith
    :type cl:fixnum
    :initform 0)
   (requirements
    :reader requirements
    :initarg :requirements
    :type C31_PathPlanner-msg:ppRequirements
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppRequirements)))
)

(cl:defclass PathPlan-request (<PathPlan-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathPlan-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathPlan-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-srv:<PathPlan-request> is deprecated: use C31_PathPlanner-srv:PathPlan-request instead.")))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-srv:map-val is deprecated.  Use C31_PathPlanner-srv:map instead.")
  (map m))

(cl:ensure-generic-function 'constraints-val :lambda-list '(m))
(cl:defmethod constraints-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-srv:constraints-val is deprecated.  Use C31_PathPlanner-srv:constraints instead.")
  (constraints m))

(cl:ensure-generic-function 'start-val :lambda-list '(m))
(cl:defmethod start-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-srv:start-val is deprecated.  Use C31_PathPlanner-srv:start instead.")
  (start m))

(cl:ensure-generic-function 'destination-val :lambda-list '(m))
(cl:defmethod destination-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-srv:destination-val is deprecated.  Use C31_PathPlanner-srv:destination instead.")
  (destination m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-srv:path-val is deprecated.  Use C31_PathPlanner-srv:path instead.")
  (path m))

(cl:ensure-generic-function 'algorith-val :lambda-list '(m))
(cl:defmethod algorith-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-srv:algorith-val is deprecated.  Use C31_PathPlanner-srv:algorith instead.")
  (algorith m))

(cl:ensure-generic-function 'requirements-val :lambda-list '(m))
(cl:defmethod requirements-val ((m <PathPlan-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-srv:requirements-val is deprecated.  Use C31_PathPlanner-srv:requirements instead.")
  (requirements m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathPlan-request>) ostream)
  "Serializes a message object of type '<PathPlan-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'map) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'constraints) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'destination) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'algorith)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'requirements) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathPlan-request>) istream)
  "Deserializes a message object of type '<PathPlan-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'map) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'constraints) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'destination) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'algorith)) (cl:read-byte istream))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'requirements) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathPlan-request>)))
  "Returns string type for a service object of type '<PathPlan-request>"
  "C31_PathPlanner/PathPlanRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathPlan-request)))
  "Returns string type for a service object of type 'PathPlan-request"
  "C31_PathPlanner/PathPlanRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathPlan-request>)))
  "Returns md5sum for a message object of type '<PathPlan-request>"
  "529cb38688d070f63410399477217988")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathPlan-request)))
  "Returns md5sum for a message object of type 'PathPlan-request"
  "529cb38688d070f63410399477217988")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathPlan-request>)))
  "Returns full string definition for message of type '<PathPlan-request>"
  (cl:format cl:nil "C31_PathPlanner/ppMap map~%C31_PathPlanner/ppConstraints constraints~%C31_PathPlanner/ppLocation start~%C31_PathPlanner/ppLocation destination~%C31_PathPlanner/ppWaypoints path~%uint8 algorith~%C31_PathPlanner/ppRequirements requirements~%~%================================================================================~%MSG: C31_PathPlanner/ppMap~%uint32 width ~%uint32 height~%uint8[] data~%float64 resolution~%C31_PathPlanner/ppPosition offset~%~%================================================================================~%MSG: C31_PathPlanner/ppPosition~%C31_PathPlanner/ppLocation location~%float64 orientation~%~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%================================================================================~%MSG: C31_PathPlanner/ppConstraints~%C31_PathPlanner/ppCorridor corridor~%C31_PathPlanner/ppRobotDimension robot~%C31_PathPlanner/ppCharge[] attractors~%C31_PathPlanner/ppCharge[] repulsors~%~%================================================================================~%MSG: C31_PathPlanner/ppCorridor~%float64 width~%================================================================================~%MSG: C31_PathPlanner/ppRobotDimension~%float64 size~%================================================================================~%MSG: C31_PathPlanner/ppCharge~%C31_PathPlanner/ppLocation location~%float64 power~%~%================================================================================~%MSG: C31_PathPlanner/ppWaypoints~%C31_PathPlanner/ppLocation[] points~%================================================================================~%MSG: C31_PathPlanner/ppRequirements~%float64 wpd~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathPlan-request)))
  "Returns full string definition for message of type 'PathPlan-request"
  (cl:format cl:nil "C31_PathPlanner/ppMap map~%C31_PathPlanner/ppConstraints constraints~%C31_PathPlanner/ppLocation start~%C31_PathPlanner/ppLocation destination~%C31_PathPlanner/ppWaypoints path~%uint8 algorith~%C31_PathPlanner/ppRequirements requirements~%~%================================================================================~%MSG: C31_PathPlanner/ppMap~%uint32 width ~%uint32 height~%uint8[] data~%float64 resolution~%C31_PathPlanner/ppPosition offset~%~%================================================================================~%MSG: C31_PathPlanner/ppPosition~%C31_PathPlanner/ppLocation location~%float64 orientation~%~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%================================================================================~%MSG: C31_PathPlanner/ppConstraints~%C31_PathPlanner/ppCorridor corridor~%C31_PathPlanner/ppRobotDimension robot~%C31_PathPlanner/ppCharge[] attractors~%C31_PathPlanner/ppCharge[] repulsors~%~%================================================================================~%MSG: C31_PathPlanner/ppCorridor~%float64 width~%================================================================================~%MSG: C31_PathPlanner/ppRobotDimension~%float64 size~%================================================================================~%MSG: C31_PathPlanner/ppCharge~%C31_PathPlanner/ppLocation location~%float64 power~%~%================================================================================~%MSG: C31_PathPlanner/ppWaypoints~%C31_PathPlanner/ppLocation[] points~%================================================================================~%MSG: C31_PathPlanner/ppRequirements~%float64 wpd~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathPlan-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'map))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'constraints))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'destination))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'requirements))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathPlan-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PathPlan-request
    (cl:cons ':map (map msg))
    (cl:cons ':constraints (constraints msg))
    (cl:cons ':start (start msg))
    (cl:cons ':destination (destination msg))
    (cl:cons ':path (path msg))
    (cl:cons ':algorith (algorith msg))
    (cl:cons ':requirements (requirements msg))
))
;//! \htmlinclude PathPlan-response.msg.html

(cl:defclass <PathPlan-response> (roslisp-msg-protocol:ros-message)
  ((path
    :reader path
    :initarg :path
    :type C31_PathPlanner-msg:ppWaypoints
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppWaypoints)))
)

(cl:defclass PathPlan-response (<PathPlan-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PathPlan-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PathPlan-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-srv:<PathPlan-response> is deprecated: use C31_PathPlanner-srv:PathPlan-response instead.")))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <PathPlan-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-srv:path-val is deprecated.  Use C31_PathPlanner-srv:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PathPlan-response>) ostream)
  "Serializes a message object of type '<PathPlan-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'path) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PathPlan-response>) istream)
  "Deserializes a message object of type '<PathPlan-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'path) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PathPlan-response>)))
  "Returns string type for a service object of type '<PathPlan-response>"
  "C31_PathPlanner/PathPlanResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathPlan-response)))
  "Returns string type for a service object of type 'PathPlan-response"
  "C31_PathPlanner/PathPlanResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PathPlan-response>)))
  "Returns md5sum for a message object of type '<PathPlan-response>"
  "529cb38688d070f63410399477217988")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PathPlan-response)))
  "Returns md5sum for a message object of type 'PathPlan-response"
  "529cb38688d070f63410399477217988")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PathPlan-response>)))
  "Returns full string definition for message of type '<PathPlan-response>"
  (cl:format cl:nil "C31_PathPlanner/ppWaypoints path~%~%~%================================================================================~%MSG: C31_PathPlanner/ppWaypoints~%C31_PathPlanner/ppLocation[] points~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PathPlan-response)))
  "Returns full string definition for message of type 'PathPlan-response"
  (cl:format cl:nil "C31_PathPlanner/ppWaypoints path~%~%~%================================================================================~%MSG: C31_PathPlanner/ppWaypoints~%C31_PathPlanner/ppLocation[] points~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PathPlan-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'path))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PathPlan-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PathPlan-response
    (cl:cons ':path (path msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PathPlan)))
  'PathPlan-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PathPlan)))
  'PathPlan-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PathPlan)))
  "Returns string type for a service object of type '<PathPlan>"
  "C31_PathPlanner/PathPlan")