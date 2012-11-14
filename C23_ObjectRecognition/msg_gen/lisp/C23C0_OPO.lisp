; Auto-generated. Do not edit!


(cl:in-package C23_ObjectRecognition-msg)


;//! \htmlinclude C23C0_OPO.msg.html

(cl:defclass <C23C0_OPO> (roslisp-msg-protocol:ros-message)
  ((position
    :reader position
    :initarg :position
    :type C23_ObjectRecognition-msg:TBD
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:TBD)))
)

(cl:defclass C23C0_OPO (<C23C0_OPO>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C23C0_OPO>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C23C0_OPO)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-msg:<C23C0_OPO> is deprecated: use C23_ObjectRecognition-msg:C23C0_OPO instead.")))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <C23C0_OPO>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:position-val is deprecated.  Use C23_ObjectRecognition-msg:position instead.")
  (position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C23C0_OPO>) ostream)
  "Serializes a message object of type '<C23C0_OPO>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C23C0_OPO>) istream)
  "Deserializes a message object of type '<C23C0_OPO>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C23C0_OPO>)))
  "Returns string type for a message object of type '<C23C0_OPO>"
  "C23_ObjectRecognition/C23C0_OPO")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23C0_OPO)))
  "Returns string type for a message object of type 'C23C0_OPO"
  "C23_ObjectRecognition/C23C0_OPO")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C23C0_OPO>)))
  "Returns md5sum for a message object of type '<C23C0_OPO>"
  "ba97431ecd8f967f52f49310867af44a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C23C0_OPO)))
  "Returns md5sum for a message object of type 'C23C0_OPO"
  "ba97431ecd8f967f52f49310867af44a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C23C0_OPO>)))
  "Returns full string definition for message of type '<C23C0_OPO>"
  (cl:format cl:nil "C23_ObjectRecognition/TBD position~%~%================================================================================~%MSG: C23_ObjectRecognition/TBD~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C23C0_OPO)))
  "Returns full string definition for message of type 'C23C0_OPO"
  (cl:format cl:nil "C23_ObjectRecognition/TBD position~%~%================================================================================~%MSG: C23_ObjectRecognition/TBD~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C23C0_OPO>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C23C0_OPO>))
  "Converts a ROS message object to a list"
  (cl:list 'C23C0_OPO
    (cl:cons ':position (position msg))
))
