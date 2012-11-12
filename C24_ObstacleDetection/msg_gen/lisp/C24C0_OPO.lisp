; Auto-generated. Do not edit!


(cl:in-package C24_ObstacleDetection-msg)


;//! \htmlinclude C24C0_OPO.msg.html

(cl:defclass <C24C0_OPO> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type C24_ObstacleDetection-msg:TBD
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:TBD)))
)

(cl:defclass C24C0_OPO (<C24C0_OPO>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C24C0_OPO>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C24C0_OPO)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C24_ObstacleDetection-msg:<C24C0_OPO> is deprecated: use C24_ObstacleDetection-msg:C24C0_OPO instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <C24C0_OPO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-msg:position-val is deprecated.  Use C24_ObstacleDetection-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C24C0_OPO>) ostream)
  "Serializes a message object of type '<C24C0_OPO>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C24C0_OPO>) istream)
  "Deserializes a message object of type '<C24C0_OPO>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C24C0_OPO>)))
  "Returns string type for a message object of type '<C24C0_OPO>"
  "C24_ObstacleDetection/C24C0_OPO")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C24C0_OPO)))
  "Returns string type for a message object of type 'C24C0_OPO"
  "C24_ObstacleDetection/C24C0_OPO")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C24C0_OPO>)))
  "Returns md5sum for a message object of type '<C24C0_OPO>"
  "ba97431ecd8f967f52f49310867af44a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C24C0_OPO)))
  "Returns md5sum for a message object of type 'C24C0_OPO"
  "ba97431ecd8f967f52f49310867af44a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C24C0_OPO>)))
  "Returns full string definition for message of type '<C24C0_OPO>"
  (cl:format cl:nil "C24_ObstacleDetection/TBD position~%~%================================================================================~%MSG: C24_ObstacleDetection/TBD~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C24C0_OPO)))
  "Returns full string definition for message of type 'C24C0_OPO"
  (cl:format cl:nil "C24_ObstacleDetection/TBD position~%~%================================================================================~%MSG: C24_ObstacleDetection/TBD~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C24C0_OPO>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C24C0_OPO>))
  "Converts a ROS message object to a list"
  (cl:list 'C24C0_OPO
    (cl:cons ':position (position msg))
))
