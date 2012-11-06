; Auto-generated. Do not edit!


(cl:in-package RobilTask-msg)


;//! \htmlinclude RobilTaskGoal.msg.html

(cl:defclass <RobilTaskGoal> (roslisp-msg-protocol:ros-message)
  ((name
    :reader name
    :initarg :name
    :type cl:string
    :initform "")
   (uid
    :reader uid
    :initarg :uid
    :type cl:string
    :initform "")
   (parameters
    :reader parameters
    :initarg :parameters
    :type cl:string
    :initform ""))
)

(cl:defclass RobilTaskGoal (<RobilTaskGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobilTaskGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobilTaskGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name RobilTask-msg:<RobilTaskGoal> is deprecated: use RobilTask-msg:RobilTaskGoal instead.")))

(cl:ensure-generic-function 'name-val :lambda-list '(m))
(cl:defmethod name-val ((m <RobilTaskGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobilTask-msg:name-val is deprecated.  Use RobilTask-msg:name instead.")
  (name m))

(cl:ensure-generic-function 'uid-val :lambda-list '(m))
(cl:defmethod uid-val ((m <RobilTaskGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobilTask-msg:uid-val is deprecated.  Use RobilTask-msg:uid instead.")
  (uid m))

(cl:ensure-generic-function 'parameters-val :lambda-list '(m))
(cl:defmethod parameters-val ((m <RobilTaskGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader RobilTask-msg:parameters-val is deprecated.  Use RobilTask-msg:parameters instead.")
  (parameters m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobilTaskGoal>) ostream)
  "Serializes a message object of type '<RobilTaskGoal>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'name))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'name))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'uid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'uid))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'parameters))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'parameters))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobilTaskGoal>) istream)
  "Deserializes a message object of type '<RobilTaskGoal>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'name) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'name) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'uid) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'uid) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'parameters) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'parameters) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobilTaskGoal>)))
  "Returns string type for a message object of type '<RobilTaskGoal>"
  "RobilTask/RobilTaskGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobilTaskGoal)))
  "Returns string type for a message object of type 'RobilTaskGoal"
  "RobilTask/RobilTaskGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobilTaskGoal>)))
  "Returns md5sum for a message object of type '<RobilTaskGoal>"
  "47a1520173b55d0b167022a5f87c0efd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobilTaskGoal)))
  "Returns md5sum for a message object of type 'RobilTaskGoal"
  "47a1520173b55d0b167022a5f87c0efd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobilTaskGoal>)))
  "Returns full string definition for message of type '<RobilTaskGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%string name~%string uid~%string parameters~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobilTaskGoal)))
  "Returns full string definition for message of type 'RobilTaskGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%# Define the goal~%string name~%string uid~%string parameters~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobilTaskGoal>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'name))
     4 (cl:length (cl:slot-value msg 'uid))
     4 (cl:length (cl:slot-value msg 'parameters))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobilTaskGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'RobilTaskGoal
    (cl:cons ':name (name msg))
    (cl:cons ':uid (uid msg))
    (cl:cons ':parameters (parameters msg))
))
