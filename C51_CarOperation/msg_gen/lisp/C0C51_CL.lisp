; Auto-generated. Do not edit!


(cl:in-package C51_CarOperation-msg)


;//! \htmlinclude C0C51_CL.msg.html

(cl:defclass <C0C51_CL> (roslisp-msg-protocol:ros-message)
  ((car
    :reader car
    :initarg :car
    :type cl:float
    :initform 0.0))
)

(cl:defclass C0C51_CL (<C0C51_CL>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C51_CL>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C51_CL)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C51_CarOperation-msg:<C0C51_CL> is deprecated: use C51_CarOperation-msg:C0C51_CL instead.")))

(cl:ensure-generic-function 'car-val :lambda-list '(m))
(cl:defmethod car-val ((m <C0C51_CL>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:car-val is deprecated.  Use C51_CarOperation-msg:car instead.")
  (car m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C51_CL>) ostream)
  "Serializes a message object of type '<C0C51_CL>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'car))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C51_CL>) istream)
  "Deserializes a message object of type '<C0C51_CL>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'car) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C51_CL>)))
  "Returns string type for a message object of type '<C0C51_CL>"
  "C51_CarOperation/C0C51_CL")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C51_CL)))
  "Returns string type for a message object of type 'C0C51_CL"
  "C51_CarOperation/C0C51_CL")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C51_CL>)))
  "Returns md5sum for a message object of type '<C0C51_CL>"
  "cb8a7131dab66e8b8913c33d75ed20ce")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C51_CL)))
  "Returns md5sum for a message object of type 'C0C51_CL"
  "cb8a7131dab66e8b8913c33d75ed20ce")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C51_CL>)))
  "Returns full string definition for message of type '<C0C51_CL>"
  (cl:format cl:nil "float32 car~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C51_CL)))
  "Returns full string definition for message of type 'C0C51_CL"
  (cl:format cl:nil "float32 car~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C51_CL>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C51_CL>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C51_CL
    (cl:cons ':car (car msg))
))
