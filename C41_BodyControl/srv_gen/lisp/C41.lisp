; Auto-generated. Do not edit!


(cl:in-package C41_BodyControl-srv)


;//! \htmlinclude C41-request.msg.html

(cl:defclass <C41-request> (roslisp-msg-protocol:ros-message)
  ((Work_mode_msg
    :reader Work_mode_msg
    :initarg :Work_mode_msg
    :type C41_BodyControl-msg:C0C41_WM
    :initform (cl:make-instance 'C41_BodyControl-msg:C0C41_WM))
   (Torque_Control_msg
    :reader Torque_Control_msg
    :initarg :Torque_Control_msg
    :type C41_BodyControl-msg:C0C41_TC
    :initform (cl:make-instance 'C41_BodyControl-msg:C0C41_TC))
   (Position_Velocity_Aceleration_Control_msg
    :reader Position_Velocity_Aceleration_Control_msg
    :initarg :Position_Velocity_Aceleration_Control_msg
    :type C41_BodyControl-msg:C0C41_PVA
    :initform (cl:make-instance 'C41_BodyControl-msg:C0C41_PVA))
   (External_Load_msg
    :reader External_Load_msg
    :initarg :External_Load_msg
    :type C41_BodyControl-msg:C0C41_LOAD
    :initform (cl:make-instance 'C41_BodyControl-msg:C0C41_LOAD)))
)

(cl:defclass C41-request (<C41-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C41-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C41-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_BodyControl-srv:<C41-request> is deprecated: use C41_BodyControl-srv:C41-request instead.")))

(cl:ensure-generic-function 'Work_mode_msg-val :lambda-list '(m))
(cl:defmethod Work_mode_msg-val ((m <C41-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-srv:Work_mode_msg-val is deprecated.  Use C41_BodyControl-srv:Work_mode_msg instead.")
  (Work_mode_msg m))

(cl:ensure-generic-function 'Torque_Control_msg-val :lambda-list '(m))
(cl:defmethod Torque_Control_msg-val ((m <C41-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-srv:Torque_Control_msg-val is deprecated.  Use C41_BodyControl-srv:Torque_Control_msg instead.")
  (Torque_Control_msg m))

(cl:ensure-generic-function 'Position_Velocity_Aceleration_Control_msg-val :lambda-list '(m))
(cl:defmethod Position_Velocity_Aceleration_Control_msg-val ((m <C41-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-srv:Position_Velocity_Aceleration_Control_msg-val is deprecated.  Use C41_BodyControl-srv:Position_Velocity_Aceleration_Control_msg instead.")
  (Position_Velocity_Aceleration_Control_msg m))

(cl:ensure-generic-function 'External_Load_msg-val :lambda-list '(m))
(cl:defmethod External_Load_msg-val ((m <C41-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-srv:External_Load_msg-val is deprecated.  Use C41_BodyControl-srv:External_Load_msg instead.")
  (External_Load_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C41-request>) ostream)
  "Serializes a message object of type '<C41-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Work_mode_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Torque_Control_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Position_Velocity_Aceleration_Control_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'External_Load_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C41-request>) istream)
  "Deserializes a message object of type '<C41-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Work_mode_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Torque_Control_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Position_Velocity_Aceleration_Control_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'External_Load_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C41-request>)))
  "Returns string type for a service object of type '<C41-request>"
  "C41_BodyControl/C41Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C41-request)))
  "Returns string type for a service object of type 'C41-request"
  "C41_BodyControl/C41Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C41-request>)))
  "Returns md5sum for a message object of type '<C41-request>"
  "e0976060194f42b49370255417cbacf7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C41-request)))
  "Returns md5sum for a message object of type 'C41-request"
  "e0976060194f42b49370255417cbacf7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C41-request>)))
  "Returns full string definition for message of type '<C41-request>"
  (cl:format cl:nil "C41_BodyControl/C0C41_WM Work_mode_msg~%C41_BodyControl/C0C41_TC Torque_Control_msg~%C41_BodyControl/C0C41_PVA Position_Velocity_Aceleration_Control_msg~%C41_BodyControl/C0C41_LOAD External_Load_msg~%~%================================================================================~%MSG: C41_BodyControl/C0C41_WM~%int32 Work_mode~%int32 Torque_Control=1~%int32 PVA_Control=2~%~%~%================================================================================~%MSG: C41_BodyControl/C0C41_TC~%float32[] TC_TORQUE~%~%================================================================================~%MSG: C41_BodyControl/C0C41_PVA~%float32[] PVA_POS~%float32[] PVA_VEL~%float32[] PVA_ACC~%~%================================================================================~%MSG: C41_BodyControl/C0C41_LOAD~%float32[] Load~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C41-request)))
  "Returns full string definition for message of type 'C41-request"
  (cl:format cl:nil "C41_BodyControl/C0C41_WM Work_mode_msg~%C41_BodyControl/C0C41_TC Torque_Control_msg~%C41_BodyControl/C0C41_PVA Position_Velocity_Aceleration_Control_msg~%C41_BodyControl/C0C41_LOAD External_Load_msg~%~%================================================================================~%MSG: C41_BodyControl/C0C41_WM~%int32 Work_mode~%int32 Torque_Control=1~%int32 PVA_Control=2~%~%~%================================================================================~%MSG: C41_BodyControl/C0C41_TC~%float32[] TC_TORQUE~%~%================================================================================~%MSG: C41_BodyControl/C0C41_PVA~%float32[] PVA_POS~%float32[] PVA_VEL~%float32[] PVA_ACC~%~%================================================================================~%MSG: C41_BodyControl/C0C41_LOAD~%float32[] Load~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C41-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Work_mode_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Torque_Control_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Position_Velocity_Aceleration_Control_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'External_Load_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C41-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C41-request
    (cl:cons ':Work_mode_msg (Work_mode_msg msg))
    (cl:cons ':Torque_Control_msg (Torque_Control_msg msg))
    (cl:cons ':Position_Velocity_Aceleration_Control_msg (Position_Velocity_Aceleration_Control_msg msg))
    (cl:cons ':External_Load_msg (External_Load_msg msg))
))
;//! \htmlinclude C41-response.msg.html

(cl:defclass <C41-response> (roslisp-msg-protocol:ros-message)
  ((Actual_torque_on_each_joint_msg
    :reader Actual_torque_on_each_joint_msg
    :initarg :Actual_torque_on_each_joint_msg
    :type C41_BodyControl-msg:C41C0_AT
    :initform (cl:make-instance 'C41_BodyControl-msg:C41C0_AT))
   (Actual_Position_Velocity_Acceleration_of_each_joint_msg
    :reader Actual_Position_Velocity_Acceleration_of_each_joint_msg
    :initarg :Actual_Position_Velocity_Acceleration_of_each_joint_msg
    :type C41_BodyControl-msg:C41C0_APVA
    :initform (cl:make-instance 'C41_BodyControl-msg:C41C0_APVA)))
)

(cl:defclass C41-response (<C41-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C41-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C41-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C41_BodyControl-srv:<C41-response> is deprecated: use C41_BodyControl-srv:C41-response instead.")))

(cl:ensure-generic-function 'Actual_torque_on_each_joint_msg-val :lambda-list '(m))
(cl:defmethod Actual_torque_on_each_joint_msg-val ((m <C41-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-srv:Actual_torque_on_each_joint_msg-val is deprecated.  Use C41_BodyControl-srv:Actual_torque_on_each_joint_msg instead.")
  (Actual_torque_on_each_joint_msg m))

(cl:ensure-generic-function 'Actual_Position_Velocity_Acceleration_of_each_joint_msg-val :lambda-list '(m))
(cl:defmethod Actual_Position_Velocity_Acceleration_of_each_joint_msg-val ((m <C41-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C41_BodyControl-srv:Actual_Position_Velocity_Acceleration_of_each_joint_msg-val is deprecated.  Use C41_BodyControl-srv:Actual_Position_Velocity_Acceleration_of_each_joint_msg instead.")
  (Actual_Position_Velocity_Acceleration_of_each_joint_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C41-response>) ostream)
  "Serializes a message object of type '<C41-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Actual_torque_on_each_joint_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Actual_Position_Velocity_Acceleration_of_each_joint_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C41-response>) istream)
  "Deserializes a message object of type '<C41-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Actual_torque_on_each_joint_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Actual_Position_Velocity_Acceleration_of_each_joint_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C41-response>)))
  "Returns string type for a service object of type '<C41-response>"
  "C41_BodyControl/C41Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C41-response)))
  "Returns string type for a service object of type 'C41-response"
  "C41_BodyControl/C41Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C41-response>)))
  "Returns md5sum for a message object of type '<C41-response>"
  "e0976060194f42b49370255417cbacf7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C41-response)))
  "Returns md5sum for a message object of type 'C41-response"
  "e0976060194f42b49370255417cbacf7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C41-response>)))
  "Returns full string definition for message of type '<C41-response>"
  (cl:format cl:nil "C41_BodyControl/C41C0_AT Actual_torque_on_each_joint_msg~%C41_BodyControl/C41C0_APVA Actual_Position_Velocity_Acceleration_of_each_joint_msg~%~%~%================================================================================~%MSG: C41_BodyControl/C41C0_AT~%float32[] Actual_torque ~%~%================================================================================~%MSG: C41_BodyControl/C41C0_APVA~%float32[] Actual_Position~%float32[] Actual_Velocity~%float32[] Actual_Acceleration ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C41-response)))
  "Returns full string definition for message of type 'C41-response"
  (cl:format cl:nil "C41_BodyControl/C41C0_AT Actual_torque_on_each_joint_msg~%C41_BodyControl/C41C0_APVA Actual_Position_Velocity_Acceleration_of_each_joint_msg~%~%~%================================================================================~%MSG: C41_BodyControl/C41C0_AT~%float32[] Actual_torque ~%~%================================================================================~%MSG: C41_BodyControl/C41C0_APVA~%float32[] Actual_Position~%float32[] Actual_Velocity~%float32[] Actual_Acceleration ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C41-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Actual_torque_on_each_joint_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Actual_Position_Velocity_Acceleration_of_each_joint_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C41-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C41-response
    (cl:cons ':Actual_torque_on_each_joint_msg (Actual_torque_on_each_joint_msg msg))
    (cl:cons ':Actual_Position_Velocity_Acceleration_of_each_joint_msg (Actual_Position_Velocity_Acceleration_of_each_joint_msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C41)))
  'C41-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C41)))
  'C41-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C41)))
  "Returns string type for a service object of type '<C41>"
  "C41_BodyControl/C41")