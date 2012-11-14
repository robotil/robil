; Auto-generated. Do not edit!


(cl:in-package C31_PathPlanner-msg)


;//! \htmlinclude ppCharge.msg.html

(cl:defclass <ppCharge> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type C31_PathPlanner-msg:ppLocation
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppLocation))
   (power
    :reader power
    :initarg :power
    :type cl:float
    :initform 0.0))
)

(cl:defclass ppCharge (<ppCharge>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ppCharge>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ppCharge)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-msg:<ppCharge> is deprecated: use C31_PathPlanner-msg:ppCharge instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <ppCharge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:location-val is deprecated.  Use C31_PathPlanner-msg:location instead.")
  (location m))

(cl:ensure-generic-function 'power-val :lambda-list '(m))
(cl:defmethod power-val ((m <ppCharge>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:power-val is deprecated.  Use C31_PathPlanner-msg:power instead.")
  (power m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ppCharge>) ostream)
  "Serializes a message object of type '<ppCharge>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'power))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ppCharge>) istream)
  "Deserializes a message object of type '<ppCharge>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'location) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'power) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ppCharge>)))
  "Returns string type for a message object of type '<ppCharge>"
  "C31_PathPlanner/ppCharge")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ppCharge)))
  "Returns string type for a message object of type 'ppCharge"
  "C31_PathPlanner/ppCharge")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ppCharge>)))
  "Returns md5sum for a message object of type '<ppCharge>"
  "4e7f73b132c0d57d62e4236bf7749719")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ppCharge)))
  "Returns md5sum for a message object of type 'ppCharge"
  "4e7f73b132c0d57d62e4236bf7749719")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ppCharge>)))
  "Returns full string definition for message of type '<ppCharge>"
  (cl:format cl:nil "C31_PathPlanner/ppLocation location~%float64 power~%~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ppCharge)))
  "Returns full string definition for message of type 'ppCharge"
  (cl:format cl:nil "C31_PathPlanner/ppLocation location~%float64 power~%~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ppCharge>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ppCharge>))
  "Converts a ROS message object to a list"
  (cl:list 'ppCharge
    (cl:cons ':location (location msg))
    (cl:cons ':power (power msg))
))
