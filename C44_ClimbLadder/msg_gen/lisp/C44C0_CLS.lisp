; Auto-generated. Do not edit!


(cl:in-package C44_ClimbLadder-msg)


;//! \htmlinclude C44C0_CLS.msg.html

(cl:defclass <C44C0_CLS> (roslisp-msg-protocol:ros-message)
  ((climb_a_ladder_success
    :reader climb_a_ladder_success
    :initarg :climb_a_ladder_success
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass C44C0_CLS (<C44C0_CLS>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C44C0_CLS>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C44C0_CLS)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C44_ClimbLadder-msg:<C44C0_CLS> is deprecated: use C44_ClimbLadder-msg:C44C0_CLS instead.")))

(cl:ensure-generic-function 'climb_a_ladder_success-val :lambda-list '(m))
(cl:defmethod climb_a_ladder_success-val ((m <C44C0_CLS>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C44_ClimbLadder-msg:climb_a_ladder_success-val is deprecated.  Use C44_ClimbLadder-msg:climb_a_ladder_success instead.")
  (climb_a_ladder_success m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C44C0_CLS>) ostream)
  "Serializes a message object of type '<C44C0_CLS>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'climb_a_ladder_success) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C44C0_CLS>) istream)
  "Deserializes a message object of type '<C44C0_CLS>"
    (cl:setf (cl:slot-value msg 'climb_a_ladder_success) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C44C0_CLS>)))
  "Returns string type for a message object of type '<C44C0_CLS>"
  "C44_ClimbLadder/C44C0_CLS")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C44C0_CLS)))
  "Returns string type for a message object of type 'C44C0_CLS"
  "C44_ClimbLadder/C44C0_CLS")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C44C0_CLS>)))
  "Returns md5sum for a message object of type '<C44C0_CLS>"
  "77d0b3fede04687d0247511e6ddd0ab1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C44C0_CLS)))
  "Returns md5sum for a message object of type 'C44C0_CLS"
  "77d0b3fede04687d0247511e6ddd0ab1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C44C0_CLS>)))
  "Returns full string definition for message of type '<C44C0_CLS>"
  (cl:format cl:nil "bool climb_a_ladder_success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C44C0_CLS)))
  "Returns full string definition for message of type 'C44C0_CLS"
  (cl:format cl:nil "bool climb_a_ladder_success~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C44C0_CLS>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C44C0_CLS>))
  "Converts a ROS message object to a list"
  (cl:list 'C44C0_CLS
    (cl:cons ':climb_a_ladder_success (climb_a_ladder_success msg))
))
