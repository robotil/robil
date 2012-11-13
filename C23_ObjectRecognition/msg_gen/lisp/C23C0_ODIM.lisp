; Auto-generated. Do not edit!


(cl:in-package C23_ObjectRecognition-msg)


;//! \htmlinclude C23C0_ODIM.msg.html

(cl:defclass <C23C0_ODIM> (roslisp-msg-protocol:ros-message)
  ((min_dimensions
    :reader min_dimensions
    :initarg :min_dimensions
    :type C23_ObjectRecognition-msg:TBD
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:TBD))
   (max_dimensions
    :reader max_dimensions
    :initarg :max_dimensions
    :type C23_ObjectRecognition-msg:TBD
    :initform (cl:make-instance 'C23_ObjectRecognition-msg:TBD)))
)

(cl:defclass C23C0_ODIM (<C23C0_ODIM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C23C0_ODIM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C23C0_ODIM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-msg:<C23C0_ODIM> is deprecated: use C23_ObjectRecognition-msg:C23C0_ODIM instead.")))

(cl:ensure-generic-function 'min_dimensions-val :lambda-list '(m))
(cl:defmethod min_dimensions-val ((m <C23C0_ODIM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:min_dimensions-val is deprecated.  Use C23_ObjectRecognition-msg:min_dimensions instead.")
  (min_dimensions m))

(cl:ensure-generic-function 'max_dimensions-val :lambda-list '(m))
(cl:defmethod max_dimensions-val ((m <C23C0_ODIM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:max_dimensions-val is deprecated.  Use C23_ObjectRecognition-msg:max_dimensions instead.")
  (max_dimensions m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C23C0_ODIM>) ostream)
  "Serializes a message object of type '<C23C0_ODIM>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'min_dimensions) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'max_dimensions) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C23C0_ODIM>) istream)
  "Deserializes a message object of type '<C23C0_ODIM>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'min_dimensions) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'max_dimensions) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C23C0_ODIM>)))
  "Returns string type for a message object of type '<C23C0_ODIM>"
  "C23_ObjectRecognition/C23C0_ODIM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23C0_ODIM)))
  "Returns string type for a message object of type 'C23C0_ODIM"
  "C23_ObjectRecognition/C23C0_ODIM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C23C0_ODIM>)))
  "Returns md5sum for a message object of type '<C23C0_ODIM>"
  "bb2056bb72057f005681e4ca255f0766")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C23C0_ODIM)))
  "Returns md5sum for a message object of type 'C23C0_ODIM"
  "bb2056bb72057f005681e4ca255f0766")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C23C0_ODIM>)))
  "Returns full string definition for message of type '<C23C0_ODIM>"
  (cl:format cl:nil "C23_ObjectRecognition/TBD min_dimensions~%C23_ObjectRecognition/TBD max_dimensions~%~%================================================================================~%MSG: C23_ObjectRecognition/TBD~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C23C0_ODIM)))
  "Returns full string definition for message of type 'C23C0_ODIM"
  (cl:format cl:nil "C23_ObjectRecognition/TBD min_dimensions~%C23_ObjectRecognition/TBD max_dimensions~%~%================================================================================~%MSG: C23_ObjectRecognition/TBD~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C23C0_ODIM>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'min_dimensions))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'max_dimensions))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C23C0_ODIM>))
  "Converts a ROS message object to a list"
  (cl:list 'C23C0_ODIM
    (cl:cons ':min_dimensions (min_dimensions msg))
    (cl:cons ':max_dimensions (max_dimensions msg))
))
