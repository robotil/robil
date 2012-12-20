; Auto-generated. Do not edit!


(cl:in-package C51_CarOperation-msg)


;//! \htmlinclude C0C51_PAR.msg.html

(cl:defclass <C0C51_PAR> (roslisp-msg-protocol:ros-message)
  ((PARV_VM
    :reader PARV_VM
    :initarg :PARV_VM
    :type cl:float
    :initform 0.0)
   (PARV_VT
    :reader PARV_VT
    :initarg :PARV_VT
    :type cl:float
    :initform 0.0))
)

(cl:defclass C0C51_PAR (<C0C51_PAR>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C51_PAR>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C51_PAR)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C51_CarOperation-msg:<C0C51_PAR> is deprecated: use C51_CarOperation-msg:C0C51_PAR instead.")))

(cl:ensure-generic-function 'PARV_VM-val :lambda-list '(m))
(cl:defmethod PARV_VM-val ((m <C0C51_PAR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:PARV_VM-val is deprecated.  Use C51_CarOperation-msg:PARV_VM instead.")
  (PARV_VM m))

(cl:ensure-generic-function 'PARV_VT-val :lambda-list '(m))
(cl:defmethod PARV_VT-val ((m <C0C51_PAR>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-msg:PARV_VT-val is deprecated.  Use C51_CarOperation-msg:PARV_VT instead.")
  (PARV_VT m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C51_PAR>) ostream)
  "Serializes a message object of type '<C0C51_PAR>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PARV_VM))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'PARV_VT))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C51_PAR>) istream)
  "Deserializes a message object of type '<C0C51_PAR>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PARV_VM) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'PARV_VT) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C51_PAR>)))
  "Returns string type for a message object of type '<C0C51_PAR>"
  "C51_CarOperation/C0C51_PAR")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C51_PAR)))
  "Returns string type for a message object of type 'C0C51_PAR"
  "C51_CarOperation/C0C51_PAR")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C51_PAR>)))
  "Returns md5sum for a message object of type '<C0C51_PAR>"
  "a14a3a4093f1ac695870fbc7ce8ae9f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C51_PAR)))
  "Returns md5sum for a message object of type 'C0C51_PAR"
  "a14a3a4093f1ac695870fbc7ce8ae9f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C51_PAR>)))
  "Returns full string definition for message of type '<C0C51_PAR>"
  (cl:format cl:nil "float32 PARV_VM~%float32 PARV_VT~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C51_PAR)))
  "Returns full string definition for message of type 'C0C51_PAR"
  (cl:format cl:nil "float32 PARV_VM~%float32 PARV_VT~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C51_PAR>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C51_PAR>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C51_PAR
    (cl:cons ':PARV_VM (PARV_VM msg))
    (cl:cons ':PARV_VT (PARV_VT msg))
))
