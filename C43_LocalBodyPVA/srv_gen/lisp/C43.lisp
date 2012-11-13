; Auto-generated. Do not edit!


(cl:in-package C43_LocalBodyPVA-srv)


;//! \htmlinclude C43-request.msg.html

(cl:defclass <C43-request> (roslisp-msg-protocol:ros-message)
  ((Selected_joint_msg
    :reader Selected_joint_msg
    :initarg :Selected_joint_msg
    :type C43_LocalBodyPVA-msg:C0C43_SJ
    :initform (cl:make-instance 'C43_LocalBodyPVA-msg:C0C43_SJ))
   (Selected_link_msg
    :reader Selected_link_msg
    :initarg :Selected_link_msg
    :type C43_LocalBodyPVA-msg:C0C43_SL
    :initform (cl:make-instance 'C43_LocalBodyPVA-msg:C0C43_SL))
   (Selected_information_msg
    :reader Selected_information_msg
    :initarg :Selected_information_msg
    :type C43_LocalBodyPVA-msg:C0C43_SI
    :initform (cl:make-instance 'C43_LocalBodyPVA-msg:C0C43_SI))
   (Selected_link_information_msg
    :reader Selected_link_information_msg
    :initarg :Selected_link_information_msg
    :type C43_LocalBodyPVA-msg:C0C43_SLI
    :initform (cl:make-instance 'C43_LocalBodyPVA-msg:C0C43_SLI)))
)

(cl:defclass C43-request (<C43-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C43-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C43-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C43_LocalBodyPVA-srv:<C43-request> is deprecated: use C43_LocalBodyPVA-srv:C43-request instead.")))

(cl:ensure-generic-function 'Selected_joint_msg-val :lambda-list '(m))
(cl:defmethod Selected_joint_msg-val ((m <C43-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-srv:Selected_joint_msg-val is deprecated.  Use C43_LocalBodyPVA-srv:Selected_joint_msg instead.")
  (Selected_joint_msg m))

(cl:ensure-generic-function 'Selected_link_msg-val :lambda-list '(m))
(cl:defmethod Selected_link_msg-val ((m <C43-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-srv:Selected_link_msg-val is deprecated.  Use C43_LocalBodyPVA-srv:Selected_link_msg instead.")
  (Selected_link_msg m))

(cl:ensure-generic-function 'Selected_information_msg-val :lambda-list '(m))
(cl:defmethod Selected_information_msg-val ((m <C43-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-srv:Selected_information_msg-val is deprecated.  Use C43_LocalBodyPVA-srv:Selected_information_msg instead.")
  (Selected_information_msg m))

(cl:ensure-generic-function 'Selected_link_information_msg-val :lambda-list '(m))
(cl:defmethod Selected_link_information_msg-val ((m <C43-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-srv:Selected_link_information_msg-val is deprecated.  Use C43_LocalBodyPVA-srv:Selected_link_information_msg instead.")
  (Selected_link_information_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C43-request>) ostream)
  "Serializes a message object of type '<C43-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Selected_joint_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Selected_link_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Selected_information_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Selected_link_information_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C43-request>) istream)
  "Deserializes a message object of type '<C43-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Selected_joint_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Selected_link_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Selected_information_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Selected_link_information_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C43-request>)))
  "Returns string type for a service object of type '<C43-request>"
  "C43_LocalBodyPVA/C43Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C43-request)))
  "Returns string type for a service object of type 'C43-request"
  "C43_LocalBodyPVA/C43Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C43-request>)))
  "Returns md5sum for a message object of type '<C43-request>"
  "8df4dbe242e088d627919891d50cae63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C43-request)))
  "Returns md5sum for a message object of type 'C43-request"
  "8df4dbe242e088d627919891d50cae63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C43-request>)))
  "Returns full string definition for message of type '<C43-request>"
  (cl:format cl:nil "C43_LocalBodyPVA/C0C43_SJ Selected_joint_msg~%C43_LocalBodyPVA/C0C43_SL Selected_link_msg~%C43_LocalBodyPVA/C0C43_SI Selected_information_msg~%C43_LocalBodyPVA/C0C43_SLI Selected_link_information_msg~%~%================================================================================~%MSG: C43_LocalBodyPVA/C0C43_SJ~%int32[] Selected_Joint~%~%================================================================================~%MSG: C43_LocalBodyPVA/C0C43_SL~%int32[] Selected_Link~%~%================================================================================~%MSG: C43_LocalBodyPVA/C0C43_SI~%bool[] Selected_Position~%bool[] Selected_Velocity~%bool[] Selected_Acceleration~%~%~%================================================================================~%MSG: C43_LocalBodyPVA/C0C43_SLI~%bool[] x_dir~%bool[] y_dir~%bool[] z_dir~%bool[] yaw~%bool[] pitch~%bool[] roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C43-request)))
  "Returns full string definition for message of type 'C43-request"
  (cl:format cl:nil "C43_LocalBodyPVA/C0C43_SJ Selected_joint_msg~%C43_LocalBodyPVA/C0C43_SL Selected_link_msg~%C43_LocalBodyPVA/C0C43_SI Selected_information_msg~%C43_LocalBodyPVA/C0C43_SLI Selected_link_information_msg~%~%================================================================================~%MSG: C43_LocalBodyPVA/C0C43_SJ~%int32[] Selected_Joint~%~%================================================================================~%MSG: C43_LocalBodyPVA/C0C43_SL~%int32[] Selected_Link~%~%================================================================================~%MSG: C43_LocalBodyPVA/C0C43_SI~%bool[] Selected_Position~%bool[] Selected_Velocity~%bool[] Selected_Acceleration~%~%~%================================================================================~%MSG: C43_LocalBodyPVA/C0C43_SLI~%bool[] x_dir~%bool[] y_dir~%bool[] z_dir~%bool[] yaw~%bool[] pitch~%bool[] roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C43-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Selected_joint_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Selected_link_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Selected_information_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Selected_link_information_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C43-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C43-request
    (cl:cons ':Selected_joint_msg (Selected_joint_msg msg))
    (cl:cons ':Selected_link_msg (Selected_link_msg msg))
    (cl:cons ':Selected_information_msg (Selected_information_msg msg))
    (cl:cons ':Selected_link_information_msg (Selected_link_information_msg msg))
))
;//! \htmlinclude C43-response.msg.html

(cl:defclass <C43-response> (roslisp-msg-protocol:ros-message)
  ((PVA_of_requested_joint_msg
    :reader PVA_of_requested_joint_msg
    :initarg :PVA_of_requested_joint_msg
    :type C43_LocalBodyPVA-msg:C43C0_JPVA
    :initform (cl:make-instance 'C43_LocalBodyPVA-msg:C43C0_JPVA))
   (PVA_of_requested_link_local_ref_frame_msg
    :reader PVA_of_requested_link_local_ref_frame_msg
    :initarg :PVA_of_requested_link_local_ref_frame_msg
    :type C43_LocalBodyPVA-msg:C43C0_LPVA
    :initform (cl:make-instance 'C43_LocalBodyPVA-msg:C43C0_LPVA)))
)

(cl:defclass C43-response (<C43-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C43-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C43-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C43_LocalBodyPVA-srv:<C43-response> is deprecated: use C43_LocalBodyPVA-srv:C43-response instead.")))

(cl:ensure-generic-function 'PVA_of_requested_joint_msg-val :lambda-list '(m))
(cl:defmethod PVA_of_requested_joint_msg-val ((m <C43-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-srv:PVA_of_requested_joint_msg-val is deprecated.  Use C43_LocalBodyPVA-srv:PVA_of_requested_joint_msg instead.")
  (PVA_of_requested_joint_msg m))

(cl:ensure-generic-function 'PVA_of_requested_link_local_ref_frame_msg-val :lambda-list '(m))
(cl:defmethod PVA_of_requested_link_local_ref_frame_msg-val ((m <C43-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-srv:PVA_of_requested_link_local_ref_frame_msg-val is deprecated.  Use C43_LocalBodyPVA-srv:PVA_of_requested_link_local_ref_frame_msg instead.")
  (PVA_of_requested_link_local_ref_frame_msg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C43-response>) ostream)
  "Serializes a message object of type '<C43-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'PVA_of_requested_joint_msg) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'PVA_of_requested_link_local_ref_frame_msg) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C43-response>) istream)
  "Deserializes a message object of type '<C43-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'PVA_of_requested_joint_msg) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'PVA_of_requested_link_local_ref_frame_msg) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C43-response>)))
  "Returns string type for a service object of type '<C43-response>"
  "C43_LocalBodyPVA/C43Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C43-response)))
  "Returns string type for a service object of type 'C43-response"
  "C43_LocalBodyPVA/C43Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C43-response>)))
  "Returns md5sum for a message object of type '<C43-response>"
  "8df4dbe242e088d627919891d50cae63")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C43-response)))
  "Returns md5sum for a message object of type 'C43-response"
  "8df4dbe242e088d627919891d50cae63")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C43-response>)))
  "Returns full string definition for message of type '<C43-response>"
  (cl:format cl:nil "C43_LocalBodyPVA/C43C0_JPVA PVA_of_requested_joint_msg~%C43_LocalBodyPVA/C43C0_LPVA PVA_of_requested_link_local_ref_frame_msg~%~%~%================================================================================~%MSG: C43_LocalBodyPVA/C43C0_JPVA~%float32[] PVA_of_requested_joint~%~%================================================================================~%MSG: C43_LocalBodyPVA/C43C0_LPVA~%float32[] PVA_requested_link_local_reference_frame~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C43-response)))
  "Returns full string definition for message of type 'C43-response"
  (cl:format cl:nil "C43_LocalBodyPVA/C43C0_JPVA PVA_of_requested_joint_msg~%C43_LocalBodyPVA/C43C0_LPVA PVA_of_requested_link_local_ref_frame_msg~%~%~%================================================================================~%MSG: C43_LocalBodyPVA/C43C0_JPVA~%float32[] PVA_of_requested_joint~%~%================================================================================~%MSG: C43_LocalBodyPVA/C43C0_LPVA~%float32[] PVA_requested_link_local_reference_frame~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C43-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'PVA_of_requested_joint_msg))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'PVA_of_requested_link_local_ref_frame_msg))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C43-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C43-response
    (cl:cons ':PVA_of_requested_joint_msg (PVA_of_requested_joint_msg msg))
    (cl:cons ':PVA_of_requested_link_local_ref_frame_msg (PVA_of_requested_link_local_ref_frame_msg msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C43)))
  'C43-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C43)))
  'C43-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C43)))
  "Returns string type for a service object of type '<C43>"
  "C43_LocalBodyPVA/C43")