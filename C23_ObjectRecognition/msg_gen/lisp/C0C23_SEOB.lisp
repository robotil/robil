; Auto-generated. Do not edit!


(cl:in-package C23_ObjectRecognition-msg)


;//! \htmlinclude C0C23_SEOB.msg.html

(cl:defclass <C0C23_SEOB> (roslisp-msg-protocol:ros-message)
  ((search_object
    :reader search_object
    :initarg :search_object
    :type cl:integer
    :initform 0))
)

(cl:defclass C0C23_SEOB (<C0C23_SEOB>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C23_SEOB>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C23_SEOB)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C23_ObjectRecognition-msg:<C0C23_SEOB> is deprecated: use C23_ObjectRecognition-msg:C0C23_SEOB instead.")))

(cl:ensure-generic-function 'search_object-val :lambda-list '(m))
(cl:defmethod search_object-val ((m <C0C23_SEOB>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C23_ObjectRecognition-msg:search_object-val is deprecated.  Use C23_ObjectRecognition-msg:search_object instead.")
  (search_object m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C0C23_SEOB>)))
    "Constants for message type '<C0C23_SEOB>"
  '((:WHEEL . 0)
    (:DOOR . 1)
    (:BREAK_PEDAL . 2)
    (:STAIR . 3)
    (:LADDER . 4)
    (:VALVE . 5)
    (:TOOL . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C0C23_SEOB)))
    "Constants for message type 'C0C23_SEOB"
  '((:WHEEL . 0)
    (:DOOR . 1)
    (:BREAK_PEDAL . 2)
    (:STAIR . 3)
    (:LADDER . 4)
    (:VALVE . 5)
    (:TOOL . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C23_SEOB>) ostream)
  "Serializes a message object of type '<C0C23_SEOB>"
  (cl:let* ((signed (cl:slot-value msg 'search_object)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C23_SEOB>) istream)
  "Deserializes a message object of type '<C0C23_SEOB>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'search_object) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C23_SEOB>)))
  "Returns string type for a message object of type '<C0C23_SEOB>"
  "C23_ObjectRecognition/C0C23_SEOB")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C23_SEOB)))
  "Returns string type for a message object of type 'C0C23_SEOB"
  "C23_ObjectRecognition/C0C23_SEOB")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C23_SEOB>)))
  "Returns md5sum for a message object of type '<C0C23_SEOB>"
  "d6a60049324b94f2246adadacb09d646")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C23_SEOB)))
  "Returns md5sum for a message object of type 'C0C23_SEOB"
  "d6a60049324b94f2246adadacb09d646")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C23_SEOB>)))
  "Returns full string definition for message of type '<C0C23_SEOB>"
  (cl:format cl:nil "int32 search_object~%int32 WHEEL=0~%int32 DOOR=1~%int32 BREAK_PEDAL=2~%int32 STAIR=3~%int32 LADDER=4~%int32 VALVE=5~%int32 TOOL=6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C23_SEOB)))
  "Returns full string definition for message of type 'C0C23_SEOB"
  (cl:format cl:nil "int32 search_object~%int32 WHEEL=0~%int32 DOOR=1~%int32 BREAK_PEDAL=2~%int32 STAIR=3~%int32 LADDER=4~%int32 VALVE=5~%int32 TOOL=6~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C23_SEOB>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C23_SEOB>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C23_SEOB
    (cl:cons ':search_object (search_object msg))
))
