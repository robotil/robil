; Auto-generated. Do not edit!


(cl:in-package C45_PostureControl-msg)


;//! \htmlinclude C22C45_SSL.msg.html

(cl:defclass <C22C45_SSL> (roslisp-msg-protocol:ros-message)
  ((surface_slope
    :reader surface_slope
    :initarg :surface_slope
    :type cl:float
    :initform 0.0))
)

(cl:defclass C22C45_SSL (<C22C45_SSL>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C22C45_SSL>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C22C45_SSL)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C45_PostureControl-msg:<C22C45_SSL> is deprecated: use C45_PostureControl-msg:C22C45_SSL instead.")))

(cl:ensure-generic-function 'surface_slope-val :lambda-list '(m))
(cl:defmethod surface_slope-val ((m <C22C45_SSL>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C45_PostureControl-msg:surface_slope-val is deprecated.  Use C45_PostureControl-msg:surface_slope instead.")
  (surface_slope m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C22C45_SSL>) ostream)
  "Serializes a message object of type '<C22C45_SSL>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'surface_slope))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C22C45_SSL>) istream)
  "Deserializes a message object of type '<C22C45_SSL>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'surface_slope) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C22C45_SSL>)))
  "Returns string type for a message object of type '<C22C45_SSL>"
  "C45_PostureControl/C22C45_SSL")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C22C45_SSL)))
  "Returns string type for a message object of type 'C22C45_SSL"
  "C45_PostureControl/C22C45_SSL")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C22C45_SSL>)))
  "Returns md5sum for a message object of type '<C22C45_SSL>"
  "dedc7080cce4f9d51c6dcb6e63148b3a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C22C45_SSL)))
  "Returns md5sum for a message object of type 'C22C45_SSL"
  "dedc7080cce4f9d51c6dcb6e63148b3a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C22C45_SSL>)))
  "Returns full string definition for message of type '<C22C45_SSL>"
  (cl:format cl:nil "float32 surface_slope~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C22C45_SSL)))
  "Returns full string definition for message of type 'C22C45_SSL"
  (cl:format cl:nil "float32 surface_slope~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C22C45_SSL>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C22C45_SSL>))
  "Converts a ROS message object to a list"
  (cl:list 'C22C45_SSL
    (cl:cons ':surface_slope (surface_slope msg))
))
