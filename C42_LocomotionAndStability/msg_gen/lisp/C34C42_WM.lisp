; Auto-generated. Do not edit!


(cl:in-package C42_LocomotionAndStability-msg)


;//! \htmlinclude C34C42_WM.msg.html

(cl:defclass <C34C42_WM> (roslisp-msg-protocol:ros-message)
  ((Work_mode
    :reader Work_mode
    :initarg :Work_mode
    :type cl:integer
    :initform 0))
)

(cl:defclass C34C42_WM (<C34C42_WM>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C34C42_WM>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C34C42_WM)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C42_LocomotionAndStability-msg:<C34C42_WM> is deprecated: use C42_LocomotionAndStability-msg:C34C42_WM instead.")))

(cl:ensure-generic-function 'Work_mode-val :lambda-list '(m))
(cl:defmethod Work_mode-val ((m <C34C42_WM>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C42_LocomotionAndStability-msg:Work_mode-val is deprecated.  Use C42_LocomotionAndStability-msg:Work_mode instead.")
  (Work_mode m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<C34C42_WM>)))
    "Constants for message type '<C34C42_WM>"
  '((:STAND_IN_PLACE . 1)
    (:DYNAMIC_BIPEDAL_WALK . 2)
    (:QUASI_STATIC_BIPEDAL_WALK . 3)
    (:CRAWL . 4)
    (:TURN_IN_PLACE . 5)
    (:SINGLE_STEP . 6)
    (:BRACE_FOR_IMACT . 7)
    (:STAND_UP . 8))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'C34C42_WM)))
    "Constants for message type 'C34C42_WM"
  '((:STAND_IN_PLACE . 1)
    (:DYNAMIC_BIPEDAL_WALK . 2)
    (:QUASI_STATIC_BIPEDAL_WALK . 3)
    (:CRAWL . 4)
    (:TURN_IN_PLACE . 5)
    (:SINGLE_STEP . 6)
    (:BRACE_FOR_IMACT . 7)
    (:STAND_UP . 8))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C34C42_WM>) ostream)
  "Serializes a message object of type '<C34C42_WM>"
  (cl:let* ((signed (cl:slot-value msg 'Work_mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C34C42_WM>) istream)
  "Deserializes a message object of type '<C34C42_WM>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'Work_mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C34C42_WM>)))
  "Returns string type for a message object of type '<C34C42_WM>"
  "C42_LocomotionAndStability/C34C42_WM")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C34C42_WM)))
  "Returns string type for a message object of type 'C34C42_WM"
  "C42_LocomotionAndStability/C34C42_WM")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C34C42_WM>)))
  "Returns md5sum for a message object of type '<C34C42_WM>"
  "34e3e7e3985262a8b8f09ced854f9e77")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C34C42_WM)))
  "Returns md5sum for a message object of type 'C34C42_WM"
  "34e3e7e3985262a8b8f09ced854f9e77")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C34C42_WM>)))
  "Returns full string definition for message of type '<C34C42_WM>"
  (cl:format cl:nil "int32 Work_mode~%int32 Stand_in_place=1~%int32 Dynamic_bipedal_walk=2~%int32 Quasi_Static_bipedal_walk=3~%int32 Crawl=4~%int32 Turn_in_place=5~%int32 Single_step=6~%int32 Brace_for_imact=7~%int32 Stand_up=8~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C34C42_WM)))
  "Returns full string definition for message of type 'C34C42_WM"
  (cl:format cl:nil "int32 Work_mode~%int32 Stand_in_place=1~%int32 Dynamic_bipedal_walk=2~%int32 Quasi_Static_bipedal_walk=3~%int32 Crawl=4~%int32 Turn_in_place=5~%int32 Single_step=6~%int32 Brace_for_imact=7~%int32 Stand_up=8~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C34C42_WM>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C34C42_WM>))
  "Converts a ROS message object to a list"
  (cl:list 'C34C42_WM
    (cl:cons ':Work_mode (Work_mode msg))
))
