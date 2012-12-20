; Auto-generated. Do not edit!


(cl:in-package C41_QuasiStaticWalking-msg)


;//! \htmlinclude QuasiStaticWalkingGoal.msg.html

(cl:defclass <QuasiStaticWalkingGoal> (roslisp-msg-protocol:ros-message)
  ((map
    :reader map
    :initarg :map
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (path
    :reader path
    :initarg :path
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass QuasiStaticWalkingGoal (<QuasiStaticWalkingGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <QuasiStaticWalkingGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'QuasiStaticWalkingGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_QuasiStaticWalking-msg:<QuasiStaticWalkingGoal> is deprecated: use C41_QuasiStaticWalking-msg:QuasiStaticWalkingGoal instead.")))

(cl:ensure-generic-function 'map-val :lambda-list '(m))
(cl:defmethod map-val ((m <QuasiStaticWalkingGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_QuasiStaticWalking-msg:map-val is deprecated.  Use C41_QuasiStaticWalking-msg:map instead.")
  (map m))

(cl:ensure-generic-function 'path-val :lambda-list '(m))
(cl:defmethod path-val ((m <QuasiStaticWalkingGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_QuasiStaticWalking-msg:path-val is deprecated.  Use C41_QuasiStaticWalking-msg:path instead.")
  (path m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <QuasiStaticWalkingGoal>) ostream)
  "Serializes a message object of type '<QuasiStaticWalkingGoal>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'map))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'map))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'path))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'path))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <QuasiStaticWalkingGoal>) istream)
  "Deserializes a message object of type '<QuasiStaticWalkingGoal>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'map) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'map)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'path) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'path)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<QuasiStaticWalkingGoal>)))
  "Returns string type for a message object of type '<QuasiStaticWalkingGoal>"
  "C41_QuasiStaticWalking/QuasiStaticWalkingGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'QuasiStaticWalkingGoal)))
  "Returns string type for a message object of type 'QuasiStaticWalkingGoal"
  "C41_QuasiStaticWalking/QuasiStaticWalkingGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<QuasiStaticWalkingGoal>)))
  "Returns md5sum for a message object of type '<QuasiStaticWalkingGoal>"
  "d06e69e9758817156f4795c53bb95594")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'QuasiStaticWalkingGoal)))
  "Returns md5sum for a message object of type 'QuasiStaticWalkingGoal"
  "d06e69e9758817156f4795c53bb95594")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<QuasiStaticWalkingGoal>)))
  "Returns full string definition for message of type '<QuasiStaticWalkingGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%float32[] map~%float32[] path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'QuasiStaticWalkingGoal)))
  "Returns full string definition for message of type 'QuasiStaticWalkingGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#goal definition~%float32[] map~%float32[] path~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <QuasiStaticWalkingGoal>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'map) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'path) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <QuasiStaticWalkingGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'QuasiStaticWalkingGoal
    (cl:cons ':map (map msg))
    (cl:cons ':path (path msg))
))
