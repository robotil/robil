; Auto-generated. Do not edit!


(cl:in-package C51_CarOperation-srv)


;//! \htmlinclude C51-request.msg.html

(cl:defclass <C51-request> (roslisp-msg-protocol:ros-message)
  ((car_class
    :reader car_class
    :initarg :car_class
    :type C51_CarOperation-msg:C0C51_CL
    :initform (cl:make-instance 'C51_CarOperation-msg:C0C51_CL))
   (start_stop_car
    :reader start_stop_car
    :initarg :start_stop_car
    :type C51_CarOperation-msg:C0C51_ST
    :initform (cl:make-instance 'C51_CarOperation-msg:C0C51_ST))
   (velocity_parameters
    :reader velocity_parameters
    :initarg :velocity_parameters
    :type C51_CarOperation-msg:C0C51_PAR
    :initform (cl:make-instance 'C51_CarOperation-msg:C0C51_PAR))
   (travel_parameters
    :reader travel_parameters
    :initarg :travel_parameters
    :type C51_CarOperation-msg:C0C51_TRA
    :initform (cl:make-instance 'C51_CarOperation-msg:C0C51_TRA)))
)

(cl:defclass C51-request (<C51-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C51-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C51-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C51_CarOperation-srv:<C51-request> is deprecated: use C51_CarOperation-srv:C51-request instead.")))

(cl:ensure-generic-function 'car_class-val :lambda-list '(m))
(cl:defmethod car_class-val ((m <C51-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-srv:car_class-val is deprecated.  Use C51_CarOperation-srv:car_class instead.")
  (car_class m))

(cl:ensure-generic-function 'start_stop_car-val :lambda-list '(m))
(cl:defmethod start_stop_car-val ((m <C51-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-srv:start_stop_car-val is deprecated.  Use C51_CarOperation-srv:start_stop_car instead.")
  (start_stop_car m))

(cl:ensure-generic-function 'velocity_parameters-val :lambda-list '(m))
(cl:defmethod velocity_parameters-val ((m <C51-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-srv:velocity_parameters-val is deprecated.  Use C51_CarOperation-srv:velocity_parameters instead.")
  (velocity_parameters m))

(cl:ensure-generic-function 'travel_parameters-val :lambda-list '(m))
(cl:defmethod travel_parameters-val ((m <C51-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-srv:travel_parameters-val is deprecated.  Use C51_CarOperation-srv:travel_parameters instead.")
  (travel_parameters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C51-request>) ostream)
  "Serializes a message object of type '<C51-request>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'car_class) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'start_stop_car) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'velocity_parameters) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'travel_parameters) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C51-request>) istream)
  "Deserializes a message object of type '<C51-request>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'car_class) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'start_stop_car) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'velocity_parameters) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'travel_parameters) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C51-request>)))
  "Returns string type for a service object of type '<C51-request>"
  "C51_CarOperation/C51Request")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C51-request)))
  "Returns string type for a service object of type 'C51-request"
  "C51_CarOperation/C51Request")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C51-request>)))
  "Returns md5sum for a message object of type '<C51-request>"
  "1034508cae6a9c4a3a01b3856e8fe54d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C51-request)))
  "Returns md5sum for a message object of type 'C51-request"
  "1034508cae6a9c4a3a01b3856e8fe54d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C51-request>)))
  "Returns full string definition for message of type '<C51-request>"
  (cl:format cl:nil "C51_CarOperation/C0C51_CL car_class~%C51_CarOperation/C0C51_ST start_stop_car~%C51_CarOperation/C0C51_PAR velocity_parameters~%C51_CarOperation/C0C51_TRA travel_parameters~%~%================================================================================~%MSG: C51_CarOperation/C0C51_CL~%float32 car~%~%================================================================================~%MSG: C51_CarOperation/C0C51_ST~%int32 car_start_stop~%int32 START_CAR=1~%int32 STOP_CAR=0~%~%================================================================================~%MSG: C51_CarOperation/C0C51_PAR~%float32 PARV_VM~%float32 PARV_VT~%~%================================================================================~%MSG: C51_CarOperation/C0C51_TRA~%string TRA_RNDF~%string TAR_MDF~%string TAR_DES~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C51-request)))
  "Returns full string definition for message of type 'C51-request"
  (cl:format cl:nil "C51_CarOperation/C0C51_CL car_class~%C51_CarOperation/C0C51_ST start_stop_car~%C51_CarOperation/C0C51_PAR velocity_parameters~%C51_CarOperation/C0C51_TRA travel_parameters~%~%================================================================================~%MSG: C51_CarOperation/C0C51_CL~%float32 car~%~%================================================================================~%MSG: C51_CarOperation/C0C51_ST~%int32 car_start_stop~%int32 START_CAR=1~%int32 STOP_CAR=0~%~%================================================================================~%MSG: C51_CarOperation/C0C51_PAR~%float32 PARV_VM~%float32 PARV_VT~%~%================================================================================~%MSG: C51_CarOperation/C0C51_TRA~%string TRA_RNDF~%string TAR_MDF~%string TAR_DES~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C51-request>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'car_class))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'start_stop_car))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'velocity_parameters))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'travel_parameters))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C51-request>))
  "Converts a ROS message object to a list"
  (cl:list 'C51-request
    (cl:cons ':car_class (car_class msg))
    (cl:cons ':start_stop_car (start_stop_car msg))
    (cl:cons ':velocity_parameters (velocity_parameters msg))
    (cl:cons ':travel_parameters (travel_parameters msg))
))
;//! \htmlinclude C51-response.msg.html

(cl:defclass <C51-response> (roslisp-msg-protocol:ros-message)
  ((Normal_abnormal_travel
    :reader Normal_abnormal_travel
    :initarg :Normal_abnormal_travel
    :type C51_CarOperation-msg:C51C0_NOR
    :initform (cl:make-instance 'C51_CarOperation-msg:C51C0_NOR))
   (Car_Position
    :reader Car_Position
    :initarg :Car_Position
    :type C51_CarOperation-msg:C51C0_OPO
    :initform (cl:make-instance 'C51_CarOperation-msg:C51C0_OPO)))
)

(cl:defclass C51-response (<C51-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C51-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C51-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C51_CarOperation-srv:<C51-response> is deprecated: use C51_CarOperation-srv:C51-response instead.")))

(cl:ensure-generic-function 'Normal_abnormal_travel-val :lambda-list '(m))
(cl:defmethod Normal_abnormal_travel-val ((m <C51-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-srv:Normal_abnormal_travel-val is deprecated.  Use C51_CarOperation-srv:Normal_abnormal_travel instead.")
  (Normal_abnormal_travel m))

(cl:ensure-generic-function 'Car_Position-val :lambda-list '(m))
(cl:defmethod Car_Position-val ((m <C51-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C51_CarOperation-srv:Car_Position-val is deprecated.  Use C51_CarOperation-srv:Car_Position instead.")
  (Car_Position m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C51-response>) ostream)
  "Serializes a message object of type '<C51-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Normal_abnormal_travel) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'Car_Position) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C51-response>) istream)
  "Deserializes a message object of type '<C51-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Normal_abnormal_travel) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'Car_Position) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C51-response>)))
  "Returns string type for a service object of type '<C51-response>"
  "C51_CarOperation/C51Response")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C51-response)))
  "Returns string type for a service object of type 'C51-response"
  "C51_CarOperation/C51Response")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C51-response>)))
  "Returns md5sum for a message object of type '<C51-response>"
  "1034508cae6a9c4a3a01b3856e8fe54d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C51-response)))
  "Returns md5sum for a message object of type 'C51-response"
  "1034508cae6a9c4a3a01b3856e8fe54d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C51-response>)))
  "Returns full string definition for message of type '<C51-response>"
  (cl:format cl:nil "C51_CarOperation/C51C0_NOR Normal_abnormal_travel~%C51_CarOperation/C51C0_OPO Car_Position~%~%~%================================================================================~%MSG: C51_CarOperation/C51C0_NOR~%int32 normal_abnormal_travel~%int32 NORMAL_TRAVEL=0~%int32 ABNORMAL_TRAVEL=1~%~%================================================================================~%MSG: C51_CarOperation/C51C0_OPO~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C51-response)))
  "Returns full string definition for message of type 'C51-response"
  (cl:format cl:nil "C51_CarOperation/C51C0_NOR Normal_abnormal_travel~%C51_CarOperation/C51C0_OPO Car_Position~%~%~%================================================================================~%MSG: C51_CarOperation/C51C0_NOR~%int32 normal_abnormal_travel~%int32 NORMAL_TRAVEL=0~%int32 ABNORMAL_TRAVEL=1~%~%================================================================================~%MSG: C51_CarOperation/C51C0_OPO~%int32 x~%int32 y~%int32 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C51-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Normal_abnormal_travel))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'Car_Position))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C51-response>))
  "Converts a ROS message object to a list"
  (cl:list 'C51-response
    (cl:cons ':Normal_abnormal_travel (Normal_abnormal_travel msg))
    (cl:cons ':Car_Position (Car_Position msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'C51)))
  'C51-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'C51)))
  'C51-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C51)))
  "Returns string type for a service object of type '<C51>"
  "C51_CarOperation/C51")