; Auto-generated. Do not edit!


(cl:in-package C31_PathPlanner-msg)


;//! \htmlinclude ppWaypoints.msg.html

(cl:defclass <ppWaypoints> (roslisp-msg-protocol:ros-message)
  ((points
    :reader points
    :initarg :points
    :type (cl:vector C31_PathPlanner-msg:ppLocation)
   :initform (cl:make-array 0 :element-type 'C31_PathPlanner-msg:ppLocation :initial-element (cl:make-instance 'C31_PathPlanner-msg:ppLocation))))
)

(cl:defclass ppWaypoints (<ppWaypoints>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ppWaypoints>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ppWaypoints)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C31_PathPlanner-msg:<ppWaypoints> is deprecated: use C31_PathPlanner-msg:ppWaypoints instead.")))

(cl:ensure-generic-function 'points-val :lambda-list '(m))
(cl:defmethod points-val ((m <ppWaypoints>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C31_PathPlanner-msg:points-val is deprecated.  Use C31_PathPlanner-msg:points instead.")
  (points m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ppWaypoints>) ostream)
  "Serializes a message object of type '<ppWaypoints>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'points))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'points))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ppWaypoints>) istream)
  "Deserializes a message object of type '<ppWaypoints>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'points) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'points)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'C31_PathPlanner-msg:ppLocation))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ppWaypoints>)))
  "Returns string type for a message object of type '<ppWaypoints>"
  "C31_PathPlanner/ppWaypoints")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ppWaypoints)))
  "Returns string type for a message object of type 'ppWaypoints"
  "C31_PathPlanner/ppWaypoints")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ppWaypoints>)))
  "Returns md5sum for a message object of type '<ppWaypoints>"
  "ffd74dfa964a04e3f477faa3684da1c6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ppWaypoints)))
  "Returns md5sum for a message object of type 'ppWaypoints"
  "ffd74dfa964a04e3f477faa3684da1c6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ppWaypoints>)))
  "Returns full string definition for message of type '<ppWaypoints>"
  (cl:format cl:nil "C31_PathPlanner/ppLocation[] points~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ppWaypoints)))
  "Returns full string definition for message of type 'ppWaypoints"
  (cl:format cl:nil "C31_PathPlanner/ppLocation[] points~%================================================================================~%MSG: C31_PathPlanner/ppLocation~%float64 lat~%float64 lon~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ppWaypoints>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'points) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ppWaypoints>))
  "Converts a ROS message object to a list"
  (cl:list 'ppWaypoints
    (cl:cons ':points (points msg))
))
