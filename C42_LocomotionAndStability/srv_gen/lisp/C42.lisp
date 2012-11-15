; Auto-generated. Do not edit!


(cl:in-package C42_LocomotionAndStability-srv)


;//! \htmlinclude C42-request.msg.html

(cl:defclass <C42-request> (roslisp-msg-protocol:ros-message)
  ((Work_mode_msg
    :reader Work_mode_msg
    :initarg :Work_mode_msg
    :type C42_LocomotionAndStability-msg:C34C42_WM
    :initform (cl:make-instance 'C42_LocomotionAndStability-msg:C34C42_WM))
   (Desired_Path_state_update_msg
    :reader Desired_Path_state_update_msg
    :initarg :Desired_Path_state_update_msg
    :type C42_LocomotionAndStability-msg:C34C42_PSU
    :initform (cl:make-instance 'C42_LocomotionAndStability-msg:C34C42_PSU)))
)

(cl:defclass C42-request (<C42-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C42-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C42-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C42_LocomotionAndStability-srv:<C42-request> is deprecated: use C42_LocomotionAndStability-srv:C42-request instead.")))

(cl:ensure-generic-function 'Work_mode_msg-val :lambda-list '(m))
(cl:defmethod Work_mode_msg-val ((m <C42-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C42_LocomotionAndStability-srv:Work_mode_msg-val is deprecated.  Use C42_LocomotionAndStability-srv:Work_mode_msg instead.")
  (Work_mode_msg m))

(cl:ensure-generic-function 'Desired_Path_state_update_msg-val :lambda-list '(m))
(cl:defmethod Desired_Path_state_update_msg-val ((m <C42-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C42_LocomotionAndStability-srv:Desired_Path_state_update_msg-val is deprecated.  Use C42_LocomotionAndStability-srv:Desired_Path_state_update_msg instead.")
  (Desired_Path_state_update_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C42-request>) ostream)
  "Serializes a message object of type '<C42-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Work_mode_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Desired_Path_state_update_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C42-request>) istream)
  "Deserializes a message object of type '<C42-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Work_mode_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Desired_Path_state_update_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C42-request>)))
  "Returns string type for a service object of type '<C42-request>"
  "C42_LocomotionAndStability/C42Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C42-request)))
  "Returns string type for a service object of type 'C42-request"
  "C42_LocomotionAndStability/C42Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C42-request>)))
  "Returns md5sum for a message object of type '<C42-request>"
  "664024711124fd2c3ff9c16f9ecfc093")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C42-request)))
  "Returns md5sum for a message object of type 'C42-request"
  "664024711124fd2c3ff9c16f9ecfc093")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C42-request>)))
  "Returns full string definition for message of type '<C42-request>"
  (cl:format cl:nil "C42_LocomotionAndStability/C34C42_WM Work_mode_msg~%C42_LocomotionAndStability/C34C42_PSU Desired_Path_state_update_msg~%~%================================================================================~%MSG: C42_LocomotionAndStability/C34C42_WM~%int32 Work_mode~%int32 Stand_in_place=1~%int32 Dynamic_bipedal_walk=2~%int32 Quasi_Static_bipedal_walk=3~%int32 Crawl=4~%int32 Turn_in_place=5~%int32 Single_step=6~%int32 Brace_for_imact=7~%int32 Stand_up=8~%~%~%================================================================================~%MSG: C42_LocomotionAndStability/C34C42_PSU~%float32[] PSU_DVEL~%float32[] PSU_VP_POS~%float32[] PSU_VP_OR~%float32[] PSU_VP_TOL~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C42-request)))
  "Returns full string definition for message of type 'C42-request"
  (cl:format cl:nil "C42_LocomotionAndStability/C34C42_WM Work_mode_msg~%C42_LocomotionAndStability/C34C42_PSU Desired_Path_state_update_msg~%~%================================================================================~%MSG: C42_LocomotionAndStability/C34C42_WM~%int32 Work_mode~%int32 Stand_in_place=1~%int32 Dynamic_bipedal_walk=2~%int32 Quasi_Static_bipedal_walk=3~%int32 Crawl=4~%int32 Turn_in_place=5~%int32 Single_step=6~%int32 Brace_for_imact=7~%int32 Stand_up=8~%~%~%================================================================================~%MSG: C42_LocomotionAndStability/C34C42_PSU~%float32[] PSU_DVEL~%float32[] PSU_VP_POS~%float32[] PSU_VP_OR~%float32[] PSU_VP_TOL~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C42-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Work_mode_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Desired_Path_state_update_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C42-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C42-request
    (cl:cons ':Work_mode_msg (Work_mode_msg msg))
    (cl:cons ':Desired_Path_state_update_msg (Desired_Path_state_update_msg msg))
))
;//! \htmlinclude C42-response.msg.html

(cl:defclass <C42-response> (roslisp-msg-protocol:ros-message)
  ((Current_work_mode_msg
    :reader Current_work_mode_msg
    :initarg :Current_work_mode_msg
    :type C42_LocomotionAndStability-msg:C42C34_CS
    :initform (cl:make-instance 'C42_LocomotionAndStability-msg:C42C34_CS))
   (Events_msg
    :reader Events_msg
    :initarg :Events_msg
    :type C42_LocomotionAndStability-msg:C42C34_EVE
    :initform (cl:make-instance 'C42_LocomotionAndStability-msg:C42C34_EVE)))
)

(cl:defclass C42-response (<C42-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C42-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C42-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C42_LocomotionAndStability-srv:<C42-response> is deprecated: use C42_LocomotionAndStability-srv:C42-response instead.")))

(cl:ensure-generic-function 'Current_work_mode_msg-val :lambda-list '(m))
(cl:defmethod Current_work_mode_msg-val ((m <C42-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C42_LocomotionAndStability-srv:Current_work_mode_msg-val is deprecated.  Use C42_LocomotionAndStability-srv:Current_work_mode_msg instead.")
  (Current_work_mode_msg m))

(cl:ensure-generic-function 'Events_msg-val :lambda-list '(m))
(cl:defmethod Events_msg-val ((m <C42-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C42_LocomotionAndStability-srv:Events_msg-val is deprecated.  Use C42_LocomotionAndStability-srv:Events_msg instead.")
  (Events_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C42-response>) ostream)
  "Serializes a message object of type '<C42-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Current_work_mode_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Events_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C42-response>) istream)
  "Deserializes a message object of type '<C42-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Current_work_mode_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Events_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C42-response>)))
  "Returns string type for a service object of type '<C42-response>"
  "C42_LocomotionAndStability/C42Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C42-response)))
  "Returns string type for a service object of type 'C42-response"
  "C42_LocomotionAndStability/C42Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C42-response>)))
  "Returns md5sum for a message object of type '<C42-response>"
  "664024711124fd2c3ff9c16f9ecfc093")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C42-response)))
  "Returns md5sum for a message object of type 'C42-response"
  "664024711124fd2c3ff9c16f9ecfc093")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C42-response>)))
  "Returns full string definition for message of type '<C42-response>"
  (cl:format cl:nil "C42_LocomotionAndStability/C42C34_CS Current_work_mode_msg~%C42_LocomotionAndStability/C42C34_EVE Events_msg~%~%~%================================================================================~%MSG: C42_LocomotionAndStability/C42C34_CS~%int32 Current_Work_mode~%~%================================================================================~%MSG: C42_LocomotionAndStability/C42C34_EVE~%int32 Events~%int32 Finished_task=1~%int32 Robot_fell=2~%int32 Performance_degradation=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C42-response)))
  "Returns full string definition for message of type 'C42-response"
  (cl:format cl:nil "C42_LocomotionAndStability/C42C34_CS Current_work_mode_msg~%C42_LocomotionAndStability/C42C34_EVE Events_msg~%~%~%================================================================================~%MSG: C42_LocomotionAndStability/C42C34_CS~%int32 Current_Work_mode~%~%================================================================================~%MSG: C42_LocomotionAndStability/C42C34_EVE~%int32 Events~%int32 Finished_task=1~%int32 Robot_fell=2~%int32 Performance_degradation=3~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C42-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Current_work_mode_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Events_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C42-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C42-response
    (cl:cons ':Current_work_mode_msg (Current_work_mode_msg msg))
    (cl:cons ':Events_msg (Events_msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C42)))
  'C42-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C42)))
  'C42-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C42)))
  "Returns string type for a service object of type '<C42>"
  "C42_LocomotionAndStability/C42")