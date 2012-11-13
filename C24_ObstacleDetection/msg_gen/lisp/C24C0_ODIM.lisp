; Auto-generated. Do not edit!


(cl:in-package C24_ObstacleDetection-msg)


;//! \htmlinclude C24C0_ODIM.msg.html

(cl:defclass <C24C0_ODIM> (roslisp-msg-protocol:ros-message)
  ((min_dimensions
    :reader min_dimensions
    :initarg :min_dimensions
    :type C24_ObstacleDetection-msg:TBD
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:TBD))
   (max_dimensions
    :reader max_dimensions
    :initarg :max_dimensions
    :type C24_ObstacleDetection-msg:TBD
    :initform (cl:make-instance 'C24_ObstacleDetection-msg:TBD)))
)

(cl:defclass C24C0_ODIM (<C24C0_ODIM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C24C0_ODIM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C24C0_ODIM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C24_ObstacleDetection-msg:<C24C0_ODIM> is deprecated: use C24_ObstacleDetection-msg:C24C0_ODIM instead.")))

(cl:ensure-generic-function 'min_dimensions-val :lambda-list '(m))
(cl:defmethod min_dimensions-val ((m <C24C0_ODIM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-msg:min_dimensions-val is deprecated.  Use C24_ObstacleDetection-msg:min_dimensions instead.")
  (min_dimensions m))

(cl:ensure-generic-function 'max_dimensions-val :lambda-list '(m))
(cl:defmethod max_dimensions-val ((m <C24C0_ODIM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-msg:max_dimensions-val is deprecated.  Use C24_ObstacleDetection-msg:max_dimensions instead.")
  (max_dimensions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C24C0_ODIM>) ostream)
  "Serializes a message object of type '<C24C0_ODIM>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_dimensions) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_dimensions) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C24C0_ODIM>) istream)
  "Deserializes a message object of type '<C24C0_ODIM>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_dimensions) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_dimensions) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C24C0_ODIM>)))
  "Returns string type for a message object of type '<C24C0_ODIM>"
  "C24_ObstacleDetection/C24C0_ODIM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C24C0_ODIM)))
  "Returns string type for a message object of type 'C24C0_ODIM"
  "C24_ObstacleDetection/C24C0_ODIM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C24C0_ODIM>)))
  "Returns md5sum for a message object of type '<C24C0_ODIM>"
  "bb2056bb72057f005681e4ca255f0766")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C24C0_ODIM)))
  "Returns md5sum for a message object of type 'C24C0_ODIM"
  "bb2056bb72057f005681e4ca255f0766")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C24C0_ODIM>)))
  "Returns full string definition for message of type '<C24C0_ODIM>"
  (cl:format cl:nil "C24_ObstacleDetection/TBD min_dimensions~%C24_ObstacleDetection/TBD max_dimensions~%~%================================================================================~%MSG: C24_ObstacleDetection/TBD~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C24C0_ODIM)))
  "Returns full string definition for message of type 'C24C0_ODIM"
  (cl:format cl:nil "C24_ObstacleDetection/TBD min_dimensions~%C24_ObstacleDetection/TBD max_dimensions~%~%================================================================================~%MSG: C24_ObstacleDetection/TBD~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C24C0_ODIM>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_dimensions))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_dimensions))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C24C0_ODIM>))
  "Converts a ROS message object to a list"
  (cl:list 'C24C0_ODIM
    (cl:cons ':min_dimensions (min_dimensions msg))
    (cl:cons ':max_dimensions (max_dimensions msg))
))
