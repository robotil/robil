; Auto-generated. Do not edit!


(cl:in-package C45_PostureControl-msg)


;//! \htmlinclude C45C0_EVE.msg.html

(cl:defclass <C45C0_EVE> (roslisp-msg-protocol:ros-message)
  ((event_succes
    :reader event_succes
    :initarg :event_succes
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass C45C0_EVE (<C45C0_EVE>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C45C0_EVE>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C45C0_EVE)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C45_PostureControl-msg:<C45C0_EVE> is deprecated: use C45_PostureControl-msg:C45C0_EVE instead.")))

(cl:ensure-generic-function 'event_succes-val :lambda-list '(m))
(cl:defmethod event_succes-val ((m <C45C0_EVE>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C45_PostureControl-msg:event_succes-val is deprecated.  Use C45_PostureControl-msg:event_succes instead.")
  (event_succes m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C45C0_EVE>) ostream)
  "Serializes a message object of type '<C45C0_EVE>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'event_succes) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C45C0_EVE>) istream)
  "Deserializes a message object of type '<C45C0_EVE>"
    (cl:setf (cl:slot-value msg 'event_succes) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C45C0_EVE>)))
  "Returns string type for a message object of type '<C45C0_EVE>"
  "C45_PostureControl/C45C0_EVE")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C45C0_EVE)))
  "Returns string type for a message object of type 'C45C0_EVE"
  "C45_PostureControl/C45C0_EVE")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C45C0_EVE>)))
  "Returns md5sum for a message object of type '<C45C0_EVE>"
  "8ea3751a36b8ebb8679ba7242b0ef34a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C45C0_EVE)))
  "Returns md5sum for a message object of type 'C45C0_EVE"
  "8ea3751a36b8ebb8679ba7242b0ef34a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C45C0_EVE>)))
  "Returns full string definition for message of type '<C45C0_EVE>"
  (cl:format cl:nil "bool event_succes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C45C0_EVE)))
  "Returns full string definition for message of type 'C45C0_EVE"
  (cl:format cl:nil "bool event_succes~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C45C0_EVE>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C45C0_EVE>))
  "Converts a ROS message object to a list"
  (cl:list 'C45C0_EVE
    (cl:cons ':event_succes (event_succes msg))
))
