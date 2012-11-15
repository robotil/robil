; Auto-generated. Do not edit!


(cl:in-package C22_GroundRecognitionAndMapping-msg)


;//! \htmlinclude C0C22_SAF.msg.html

(cl:defclass <C0C22_SAF> (roslisp-msg-protocol:ros-message)
  ((safety_req
    :reader safety_req
    :initarg :safety_req
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C22_SAF (<C0C22_SAF>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C22_SAF>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C22_SAF)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C22_GroundRecognitionAndMapping-msg:<C0C22_SAF> is deprecated: use C22_GroundRecognitionAndMapping-msg:C0C22_SAF instead.")))

(cl:ensure-generic-function 'safety_req-val :lambda-list '(m))
(cl:defmethod safety_req-val ((m <C0C22_SAF>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C22_GroundRecognitionAndMapping-msg:safety_req-val is deprecated.  Use C22_GroundRecognitionAndMapping-msg:safety_req instead.")
  (safety_req m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C22_SAF>) ostream)
  "Serializes a message object of type '<C0C22_SAF>"
  (cl:let* ((signed (cl:slot-value msg 'safety_req)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C22_SAF>) istream)
  "Deserializes a message object of type '<C0C22_SAF>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'safety_req) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C22_SAF>)))
  "Returns string type for a message object of type '<C0C22_SAF>"
  "C22_GroundRecognitionAndMapping/C0C22_SAF")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C22_SAF)))
  "Returns string type for a message object of type 'C0C22_SAF"
  "C22_GroundRecognitionAndMapping/C0C22_SAF")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C22_SAF>)))
  "Returns md5sum for a message object of type '<C0C22_SAF>"
  "e8a3d2c0b93070789c8ad4302af18ae8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C22_SAF)))
  "Returns md5sum for a message object of type 'C0C22_SAF"
  "e8a3d2c0b93070789c8ad4302af18ae8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C22_SAF>)))
  "Returns full string definition for message of type '<C0C22_SAF>"
  (cl:format cl:nil "int32 safety_req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C22_SAF)))
  "Returns full string definition for message of type 'C0C22_SAF"
  (cl:format cl:nil "int32 safety_req~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C22_SAF>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C22_SAF>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C22_SAF
    (cl:cons ':safety_req (safety_req msg))
))
