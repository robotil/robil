; Auto-generated. Do not edit!


(cl:in-package C21_VisionAndLidar-msg)


;//! \htmlinclude C0C21_CAM.msg.html

(cl:defclass <C0C21_CAM> (roslisp-msg-protocol:ros-message)
  ((frameRatePerSec
    :reader frameRatePerSec
    :initarg :frameRatePerSec
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C21_CAM (<C0C21_CAM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C21_CAM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C21_CAM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C21_VisionAndLidar-msg:<C0C21_CAM> is deprecated: use C21_VisionAndLidar-msg:C0C21_CAM instead.")))

(cl:ensure-generic-function 'frameRatePerSec-val :lambda-list '(m))
(cl:defmethod frameRatePerSec-val ((m <C0C21_CAM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-msg:frameRatePerSec-val is deprecated.  Use C21_VisionAndLidar-msg:frameRatePerSec instead.")
  (frameRatePerSec m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C21_CAM>) ostream)
  "Serializes a message object of type '<C0C21_CAM>"
  (cl:let* ((signed (cl:slot-value msg 'frameRatePerSec)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C21_CAM>) istream)
  "Deserializes a message object of type '<C0C21_CAM>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'frameRatePerSec) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C21_CAM>)))
  "Returns string type for a message object of type '<C0C21_CAM>"
  "C21_VisionAndLidar/C0C21_CAM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C21_CAM)))
  "Returns string type for a message object of type 'C0C21_CAM"
  "C21_VisionAndLidar/C0C21_CAM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C21_CAM>)))
  "Returns md5sum for a message object of type '<C0C21_CAM>"
  "0556698a1696c5e5a613abb4fe2bb569")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C21_CAM)))
  "Returns md5sum for a message object of type 'C0C21_CAM"
  "0556698a1696c5e5a613abb4fe2bb569")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C21_CAM>)))
  "Returns full string definition for message of type '<C0C21_CAM>"
  (cl:format cl:nil "int32 frameRatePerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C21_CAM)))
  "Returns full string definition for message of type 'C0C21_CAM"
  (cl:format cl:nil "int32 frameRatePerSec~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C21_CAM>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C21_CAM>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C21_CAM
    (cl:cons ':frameRatePerSec (frameRatePerSec msg))
))
