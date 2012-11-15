; Auto-generated. Do not edit!


(cl:in-package C21_VisionAndLidar-msg)


;//! \htmlinclude C0C21_RES.msg.html

(cl:defclass <C0C21_RES> (roslisp-msg-protocol:ros-message)
  ((resulution_width
    :reader resulution_width
    :initarg :resulution_width
    :type cl:integer
    :initform 0)
   (resulution_height
    :reader resulution_height
    :initarg :resulution_height
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C21_RES (<C0C21_RES>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C21_RES>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C21_RES)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C21_VisionAndLidar-msg:<C0C21_RES> is deprecated: use C21_VisionAndLidar-msg:C0C21_RES instead.")))

(cl:ensure-generic-function 'resulution_width-val :lambda-list '(m))
(cl:defmethod resulution_width-val ((m <C0C21_RES>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-msg:resulution_width-val is deprecated.  Use C21_VisionAndLidar-msg:resulution_width instead.")
  (resulution_width m))

(cl:ensure-generic-function 'resulution_height-val :lambda-list '(m))
(cl:defmethod resulution_height-val ((m <C0C21_RES>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C21_VisionAndLidar-msg:resulution_height-val is deprecated.  Use C21_VisionAndLidar-msg:resulution_height instead.")
  (resulution_height m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C21_RES>) ostream)
  "Serializes a message object of type '<C0C21_RES>"
  (cl:let* ((signed (cl:slot-value msg 'resulution_width)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'resulution_height)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C21_RES>) istream)
  "Deserializes a message object of type '<C0C21_RES>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resulution_width) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'resulution_height) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C21_RES>)))
  "Returns string type for a message object of type '<C0C21_RES>"
  "C21_VisionAndLidar/C0C21_RES")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C21_RES)))
  "Returns string type for a message object of type 'C0C21_RES"
  "C21_VisionAndLidar/C0C21_RES")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C21_RES>)))
  "Returns md5sum for a message object of type '<C0C21_RES>"
  "aa40cbd8efd8ee1390818a480235f8ed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C21_RES)))
  "Returns md5sum for a message object of type 'C0C21_RES"
  "aa40cbd8efd8ee1390818a480235f8ed")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C21_RES>)))
  "Returns full string definition for message of type '<C0C21_RES>"
  (cl:format cl:nil "int32 resulution_width~%int32 resulution_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C21_RES)))
  "Returns full string definition for message of type 'C0C21_RES"
  (cl:format cl:nil "int32 resulution_width~%int32 resulution_height~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C21_RES>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C21_RES>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C21_RES
    (cl:cons ':resulution_width (resulution_width msg))
    (cl:cons ':resulution_height (resulution_height msg))
))
