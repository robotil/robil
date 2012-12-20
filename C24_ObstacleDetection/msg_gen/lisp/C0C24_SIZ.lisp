; Auto-generated. Do not edit!


(cl:in-package C24_ObstacleDetection-msg)


;//! \htmlinclude C0C24_SIZ.msg.html

(cl:defclass <C0C24_SIZ> (roslisp-msg-protocol:ros-message)
  ((image_width
    :reader image_width
    :initarg :image_width
    :type cl:integer
    :initform 0)
   (image_height
    :reader image_height
    :initarg :image_height
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C24_SIZ (<C0C24_SIZ>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C24_SIZ>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C24_SIZ)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C24_ObstacleDetection-msg:<C0C24_SIZ> is deprecated: use C24_ObstacleDetection-msg:C0C24_SIZ instead.")))

(cl:ensure-generic-function 'image_width-val :lambda-list '(m))
(cl:defmethod image_width-val ((m <C0C24_SIZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-msg:image_width-val is deprecated.  Use C24_ObstacleDetection-msg:image_width instead.")
  (image_width m))

(cl:ensure-generic-function 'image_height-val :lambda-list '(m))
(cl:defmethod image_height-val ((m <C0C24_SIZ>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C24_ObstacleDetection-msg:image_height-val is deprecated.  Use C24_ObstacleDetection-msg:image_height instead.")
  (image_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C24_SIZ>) ostream)
  "Serializes a message object of type '<C0C24_SIZ>"
  (cl:let* ((signed (cl:slot-value msg 'image_width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'image_height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C24_SIZ>) istream)
  "Deserializes a message object of type '<C0C24_SIZ>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'image_height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C24_SIZ>)))
  "Returns string type for a message object of type '<C0C24_SIZ>"
  "C24_ObstacleDetection/C0C24_SIZ")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C24_SIZ)))
  "Returns string type for a message object of type 'C0C24_SIZ"
  "C24_ObstacleDetection/C0C24_SIZ")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C24_SIZ>)))
  "Returns md5sum for a message object of type '<C0C24_SIZ>"
  "55b10cd20a3d246968c1e93cc4a92af6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C24_SIZ)))
  "Returns md5sum for a message object of type 'C0C24_SIZ"
  "55b10cd20a3d246968c1e93cc4a92af6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C24_SIZ>)))
  "Returns full string definition for message of type '<C0C24_SIZ>"
  (cl:format cl:nil "int32 image_width~%int32 image_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C24_SIZ)))
  "Returns full string definition for message of type 'C0C24_SIZ"
  (cl:format cl:nil "int32 image_width~%int32 image_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C24_SIZ>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C24_SIZ>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C24_SIZ
    (cl:cons ':image_width (image_width msg))
    (cl:cons ':image_height (image_height msg))
))
