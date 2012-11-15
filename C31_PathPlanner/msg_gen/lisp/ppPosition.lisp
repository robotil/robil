; Auto-generated. Do not edit!


(cl:in-package C31_PathPlanner-msg)


;//! \htmlinclude ppPosition.msg.html

(cl:defclass <ppPosition> (roslisp-msg-protocol:ros-message)
  ((location
    :reader location
    :initarg :location
    :type C31_PathPlanner-msg:ppLocation
    :initform (cl:make-instance 'C31_PathPlanner-msg:ppLocation))
   (orientation
    :reader orientation
    :initarg :orientation
    :type cl:float
    :initform 0.0))
)

(cl:defclass ppPosition (<ppPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ppPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ppPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-msg:<ppPosition> is deprecated: use C31_PathPlanner-msg:ppPosition instead.")))

(cl:ensure-generic-function 'location-val :lambda-list '(m))
(cl:defmethod location-val ((m <ppPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:location-val is deprecated.  Use C31_PathPlanner-msg:location instead.")
  (location m))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <ppPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:orientation-val is deprecated.  Use C31_PathPlanner-msg:orientation instead.")
  (orientation m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ppPosition>) ostream)
  "Serializes a message object of type '<ppPosition>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'location) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'orientation))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ppPosition>) istream)
  "Deserializes a message object of type '<ppPosition>"
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
    (cl:setf (cl:slot-value msg 'orientation) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ppPosition>)))
  "Returns string type for a message object of type '<ppPosition>"
  "C31_PathPlanner/ppPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ppPosition)))
  "Returns string type for a message object of type 'ppPosition"
  "C31_PathPlanner/ppPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ppPosition>)))
  "Returns md5sum for a message object of type '<ppPosition>"
  "9befbb2fd26d643952bfcff3d40633ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ppPosition)))
  "Returns md5sum for a message object of type 'ppPosition"
  "9befbb2fd26d643952bfcff3d40633ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ppPosition>)))
  "Returns full string definition for message of type '<ppPosition>"
  (cl:format cl:nil "C31_PathPlanner/ppLocation location~%float64 orientation~%~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ppPosition)))
  "Returns full string definition for message of type 'ppPosition"
  (cl:format cl:nil "C31_PathPlanner/ppLocation location~%float64 orientation~%~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ppPosition>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'location))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ppPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'ppPosition
    (cl:cons ':location (location msg))
    (cl:cons ':orientation (orientation msg))
))
