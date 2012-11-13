; Auto-generated. Do not edit!


(cl:in-package C43_LocalBodyPVA-msg)


;//! \htmlinclude C0C43_SI.msg.html

(cl:defclass <C0C43_SI> (roslisp-msg-protocol:ros-message)
  ((Selected_Position
    :reader Selected_Position
    :initarg :Selected_Position
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (Selected_Velocity
    :reader Selected_Velocity
    :initarg :Selected_Velocity
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (Selected_Acceleration
    :reader Selected_Acceleration
    :initarg :Selected_Acceleration
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass C0C43_SI (<C0C43_SI>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C43_SI>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C43_SI)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C43_LocalBodyPVA-msg:<C0C43_SI> is deprecated: use C43_LocalBodyPVA-msg:C0C43_SI instead.")))

(cl:ensure-generic-function 'Selected_Position-val :lambda-list '(m))
(cl:defmethod Selected_Position-val ((m <C0C43_SI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:Selected_Position-val is deprecated.  Use C43_LocalBodyPVA-msg:Selected_Position instead.")
  (Selected_Position m))

(cl:ensure-generic-function 'Selected_Velocity-val :lambda-list '(m))
(cl:defmethod Selected_Velocity-val ((m <C0C43_SI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:Selected_Velocity-val is deprecated.  Use C43_LocalBodyPVA-msg:Selected_Velocity instead.")
  (Selected_Velocity m))

(cl:ensure-generic-function 'Selected_Acceleration-val :lambda-list '(m))
(cl:defmethod Selected_Acceleration-val ((m <C0C43_SI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:Selected_Acceleration-val is deprecated.  Use C43_LocalBodyPVA-msg:Selected_Acceleration instead.")
  (Selected_Acceleration m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C43_SI>) ostream)
  "Serializes a message object of type '<C0C43_SI>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Selected_Position))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'Selected_Position))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Selected_Velocity))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'Selected_Velocity))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'Selected_Acceleration))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'Selected_Acceleration))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C43_SI>) istream)
  "Deserializes a message object of type '<C0C43_SI>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Selected_Position) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Selected_Position)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Selected_Velocity) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Selected_Velocity)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'Selected_Acceleration) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'Selected_Acceleration)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C43_SI>)))
  "Returns string type for a message object of type '<C0C43_SI>"
  "C43_LocalBodyPVA/C0C43_SI")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C43_SI)))
  "Returns string type for a message object of type 'C0C43_SI"
  "C43_LocalBodyPVA/C0C43_SI")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C43_SI>)))
  "Returns md5sum for a message object of type '<C0C43_SI>"
  "1a391a2d62a6c96b6facd69f878ed6a5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C43_SI)))
  "Returns md5sum for a message object of type 'C0C43_SI"
  "1a391a2d62a6c96b6facd69f878ed6a5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C43_SI>)))
  "Returns full string definition for message of type '<C0C43_SI>"
  (cl:format cl:nil "bool[] Selected_Position~%bool[] Selected_Velocity~%bool[] Selected_Acceleration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C43_SI)))
  "Returns full string definition for message of type 'C0C43_SI"
  (cl:format cl:nil "bool[] Selected_Position~%bool[] Selected_Velocity~%bool[] Selected_Acceleration~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C43_SI>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Selected_Position) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Selected_Velocity) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'Selected_Acceleration) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C43_SI>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C43_SI
    (cl:cons ':Selected_Position (Selected_Position msg))
    (cl:cons ':Selected_Velocity (Selected_Velocity msg))
    (cl:cons ':Selected_Acceleration (Selected_Acceleration msg))
))
