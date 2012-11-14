; Auto-generated. Do not edit!


(cl:in-package C0_RobilTask-msg)


;//! \htmlinclude RobilTaskAction.msg.html

(cl:defclass <RobilTaskAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type C0_RobilTask-msg:RobilTaskActionGoal
    :initform (cl:make-instance 'C0_RobilTask-msg:RobilTaskActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type C0_RobilTask-msg:RobilTaskActionResult
    :initform (cl:make-instance 'C0_RobilTask-msg:RobilTaskActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type C0_RobilTask-msg:RobilTaskActionFeedback
    :initform (cl:make-instance 'C0_RobilTask-msg:RobilTaskActionFeedback)))
)

(cl:defclass RobilTaskAction (<RobilTaskAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobilTaskAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobilTaskAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C0_RobilTask-msg:<RobilTaskAction> is deprecated: use C0_RobilTask-msg:RobilTaskAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <RobilTaskAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C0_RobilTask-msg:action_goal-val is deprecated.  Use C0_RobilTask-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <RobilTaskAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C0_RobilTask-msg:action_result-val is deprecated.  Use C0_RobilTask-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <RobilTaskAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C0_RobilTask-msg:action_feedback-val is deprecated.  Use C0_RobilTask-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobilTaskAction>) ostream)
  "Serializes a message object of type '<RobilTaskAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobilTaskAction>) istream)
  "Deserializes a message object of type '<RobilTaskAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobilTaskAction>)))
  "Returns string type for a message object of type '<RobilTaskAction>"
  "C0_RobilTask/RobilTaskAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobilTaskAction)))
  "Returns string type for a message object of type 'RobilTaskAction"
  "C0_RobilTask/RobilTaskAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobilTaskAction>)))
  "Returns md5sum for a message object of type '<RobilTaskAction>"
  "9c2334a3537644a8b893b0a564150ac0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobilTaskAction)))
  "Returns md5sum for a message object of type 'RobilTaskAction"
  "9c2334a3537644a8b893b0a564150ac0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobilTaskAction>)))
  "Returns full string definition for message of type '<RobilTaskAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%RobilTaskActionGoal action_goal~%RobilTaskActionResult action_result~%RobilTaskActionFeedback action_feedback~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%RobilTaskGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%string name~%string uid~%string parameters~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%RobilTaskResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%int32 success~%string description~%string plan~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%RobilTaskFeedback feedback~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define a feedback message~%float32 complete~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobilTaskAction)))
  "Returns full string definition for message of type 'RobilTaskAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%RobilTaskActionGoal action_goal~%RobilTaskActionResult action_result~%RobilTaskActionFeedback action_feedback~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%RobilTaskGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%string name~%string uid~%string parameters~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%RobilTaskResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the result~%int32 success~%string description~%string plan~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%RobilTaskFeedback feedback~%~%================================================================================~%MSG: C0_RobilTask/RobilTaskFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define a feedback message~%float32 complete~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobilTaskAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobilTaskAction>))
  "Converts a ROS message object to a list"
  (cl:list 'RobilTaskAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
