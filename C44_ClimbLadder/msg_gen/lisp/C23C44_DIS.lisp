; Auto-generated. Do not edit!


(cl:in-package C44_ClimbLadder-msg)


;//! \htmlinclude C23C44_DIS.msg.html

(cl:defclass <C23C44_DIS> (roslisp-msg-protocol:ros-message)
  ((distance_between_steps
    :reader distance_between_steps
    :initarg :distance_between_steps
    :type cl:float
    :initform 0.0))
)

(cl:defclass C23C44_DIS (<C23C44_DIS>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C23C44_DIS>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C23C44_DIS)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C44_ClimbLadder-msg:<C23C44_DIS> is deprecated: use C44_ClimbLadder-msg:C23C44_DIS instead.")))

(cl:ensure-generic-function 'distance_between_steps-val :lambda-list '(m))
(cl:defmethod distance_between_steps-val ((m <C23C44_DIS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C44_ClimbLadder-msg:distance_between_steps-val is deprecated.  Use C44_ClimbLadder-msg:distance_between_steps instead.")
  (distance_between_steps m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C23C44_DIS>) ostream)
  "Serializes a message object of type '<C23C44_DIS>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance_between_steps))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C23C44_DIS>) istream)
  "Deserializes a message object of type '<C23C44_DIS>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance_between_steps) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C23C44_DIS>)))
  "Returns string type for a message object of type '<C23C44_DIS>"
  "C44_ClimbLadder/C23C44_DIS")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C23C44_DIS)))
  "Returns string type for a message object of type 'C23C44_DIS"
  "C44_ClimbLadder/C23C44_DIS")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C23C44_DIS>)))
  "Returns md5sum for a message object of type '<C23C44_DIS>"
  "db6da7b2192a8d3d06201358878d51ad")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C23C44_DIS)))
  "Returns md5sum for a message object of type 'C23C44_DIS"
  "db6da7b2192a8d3d06201358878d51ad")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C23C44_DIS>)))
  "Returns full string definition for message of type '<C23C44_DIS>"
  (cl:format cl:nil "float32 distance_between_steps~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C23C44_DIS)))
  "Returns full string definition for message of type 'C23C44_DIS"
  (cl:format cl:nil "float32 distance_between_steps~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C23C44_DIS>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C23C44_DIS>))
  "Converts a ROS message object to a list"
  (cl:list 'C23C44_DIS
    (cl:cons ':distance_between_steps (distance_between_steps msg))
))
