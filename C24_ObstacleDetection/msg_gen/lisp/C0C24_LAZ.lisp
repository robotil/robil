; Auto-generated. Do not edit!


(cl:in-package C24_ObstacleDetection-msg)


;//! \htmlinclude C0C24_LAZ.msg.html

(cl:defclass <C0C24_LAZ> (roslisp-msg-protocol:ros-message)
  ((sampleRatePerSec
    :reader sampleRatePerSec
    :initarg :sampleRatePerSec
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C24_LAZ (<C0C24_LAZ>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C24_LAZ>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C24_LAZ)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C24_ObstacleDetection-msg:<C0C24_LAZ> is deprecated: use C24_ObstacleDetection-msg:C0C24_LAZ instead.")))

(cl:ensure-generic-function 'sampleRatePerSec-val :lambda-list '(m))
(cl:defmethod sampleRatePerSec-val ((m <C0C24_LAZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-msg:sampleRatePerSec-val is deprecated.  Use C24_ObstacleDetection-msg:sampleRatePerSec instead.")
  (sampleRatePerSec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C24_LAZ>) ostream)
  "Serializes a message object of type '<C0C24_LAZ>"
  (cl:let* ((signed (cl:slot-value msg 'sampleRatePerSec)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C24_LAZ>) istream)
  "Deserializes a message object of type '<C0C24_LAZ>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'sampleRatePerSec) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C24_LAZ>)))
  "Returns string type for a message object of type '<C0C24_LAZ>"
  "C24_ObstacleDetection/C0C24_LAZ")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C24_LAZ)))
  "Returns string type for a message object of type 'C0C24_LAZ"
  "C24_ObstacleDetection/C0C24_LAZ")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C24_LAZ>)))
  "Returns md5sum for a message object of type '<C0C24_LAZ>"
  "e2a080ff15f5786b0e2a8ea153d8e1bb")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C24_LAZ)))
  "Returns md5sum for a message object of type 'C0C24_LAZ"
  "e2a080ff15f5786b0e2a8ea153d8e1bb")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C24_LAZ>)))
  "Returns full string definition for message of type '<C0C24_LAZ>"
  (cl:format cl:nil "int32 sampleRatePerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C24_LAZ)))
  "Returns full string definition for message of type 'C0C24_LAZ"
  (cl:format cl:nil "int32 sampleRatePerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C24_LAZ>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C24_LAZ>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C24_LAZ
    (cl:cons ':sampleRatePerSec (sampleRatePerSec msg))
))
