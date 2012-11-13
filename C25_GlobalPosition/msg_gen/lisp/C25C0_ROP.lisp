; Auto-generated. Do not edit!


(cl:in-package C25_GlobalPosition-msg)


;//! \htmlinclude C25C0_ROP.msg.html

(cl:defclass <C25C0_ROP> (roslisp-msg-protocol:ros-message)
  ((min_dimensions
    :reader min_dimensions
    :initarg :min_dimensions
    :type C25_GlobalPosition-msg:UTM
    :initform (cl:make-instance 'C25_GlobalPosition-msg:UTM)))
)

(cl:defclass C25C0_ROP (<C25C0_ROP>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C25C0_ROP>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C25C0_ROP)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C25_GlobalPosition-msg:<C25C0_ROP> is deprecated: use C25_GlobalPosition-msg:C25C0_ROP instead.")))

(cl:ensure-generic-function 'min_dimensions-val :lambda-list '(m))
(cl:defmethod min_dimensions-val ((m <C25C0_ROP>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C25_GlobalPosition-msg:min_dimensions-val is deprecated.  Use C25_GlobalPosition-msg:min_dimensions instead.")
  (min_dimensions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C25C0_ROP>) ostream)
  "Serializes a message object of type '<C25C0_ROP>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_dimensions) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C25C0_ROP>) istream)
  "Deserializes a message object of type '<C25C0_ROP>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_dimensions) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C25C0_ROP>)))
  "Returns string type for a message object of type '<C25C0_ROP>"
  "C25_GlobalPosition/C25C0_ROP")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C25C0_ROP)))
  "Returns string type for a message object of type 'C25C0_ROP"
  "C25_GlobalPosition/C25C0_ROP")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C25C0_ROP>)))
  "Returns md5sum for a message object of type '<C25C0_ROP>"
  "be84340d122053fde2ff77602328c75b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C25C0_ROP)))
  "Returns md5sum for a message object of type 'C25C0_ROP"
  "be84340d122053fde2ff77602328c75b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C25C0_ROP>)))
  "Returns full string definition for message of type '<C25C0_ROP>"
  (cl:format cl:nil "C25_GlobalPosition/UTM min_dimensions~%~%~%================================================================================~%MSG: C25_GlobalPosition/UTM~%int32 zone~%int64 easting~%int64 northing~%int32 hemisphere~%int32 NORTHERN_HEMISPHERE=0~%int32 SOUTHERN_HEMISPHERE=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C25C0_ROP)))
  "Returns full string definition for message of type 'C25C0_ROP"
  (cl:format cl:nil "C25_GlobalPosition/UTM min_dimensions~%~%~%================================================================================~%MSG: C25_GlobalPosition/UTM~%int32 zone~%int64 easting~%int64 northing~%int32 hemisphere~%int32 NORTHERN_HEMISPHERE=0~%int32 SOUTHERN_HEMISPHERE=1~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C25C0_ROP>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_dimensions))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C25C0_ROP>))
  "Converts a ROS message object to a list"
  (cl:list 'C25C0_ROP
    (cl:cons ':min_dimensions (min_dimensions msg))
))
