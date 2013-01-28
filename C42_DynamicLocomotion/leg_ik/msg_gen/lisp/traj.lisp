; Auto-generated. Do not edit!


(cl:in-package leg_ik-msg)


;//! \htmlinclude traj.msg.html

(cl:defclass <traj> (roslisp-msg-protocol:ros-message)
  ((COMx
    :reader COMx
    :initarg :COMx
    :type cl:float
    :initform 0.0)
   (COMy
    :reader COMy
    :initarg :COMy
    :type cl:float
    :initform 0.0)
   (COMz
    :reader COMz
    :initarg :COMz
    :type cl:float
    :initform 0.0)
   (Swing_x
    :reader Swing_x
    :initarg :Swing_x
    :type cl:float
    :initform 0.0)
   (Swing_y
    :reader Swing_y
    :initarg :Swing_y
    :type cl:float
    :initform 0.0)
   (Swing_z
    :reader Swing_z
    :initarg :Swing_z
    :type cl:float
    :initform 0.0)
   (leg
    :reader leg
    :initarg :leg
    :type cl:integer
    :initform 0))
)

(cl:defclass traj (<traj>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <traj>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'traj)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name leg_ik-msg:<traj> is deprecated: use leg_ik-msg:traj instead.")))

(cl:ensure-generic-function 'COMx-val :lambda-list '(m))
(cl:defmethod COMx-val ((m <traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-msg:COMx-val is deprecated.  Use leg_ik-msg:COMx instead.")
  (COMx m))

(cl:ensure-generic-function 'COMy-val :lambda-list '(m))
(cl:defmethod COMy-val ((m <traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-msg:COMy-val is deprecated.  Use leg_ik-msg:COMy instead.")
  (COMy m))

(cl:ensure-generic-function 'COMz-val :lambda-list '(m))
(cl:defmethod COMz-val ((m <traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-msg:COMz-val is deprecated.  Use leg_ik-msg:COMz instead.")
  (COMz m))

(cl:ensure-generic-function 'Swing_x-val :lambda-list '(m))
(cl:defmethod Swing_x-val ((m <traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-msg:Swing_x-val is deprecated.  Use leg_ik-msg:Swing_x instead.")
  (Swing_x m))

(cl:ensure-generic-function 'Swing_y-val :lambda-list '(m))
(cl:defmethod Swing_y-val ((m <traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-msg:Swing_y-val is deprecated.  Use leg_ik-msg:Swing_y instead.")
  (Swing_y m))

(cl:ensure-generic-function 'Swing_z-val :lambda-list '(m))
(cl:defmethod Swing_z-val ((m <traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-msg:Swing_z-val is deprecated.  Use leg_ik-msg:Swing_z instead.")
  (Swing_z m))

(cl:ensure-generic-function 'leg-val :lambda-list '(m))
(cl:defmethod leg-val ((m <traj>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader leg_ik-msg:leg-val is deprecated.  Use leg_ik-msg:leg instead.")
  (leg m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <traj>) ostream)
  "Serializes a message object of type '<traj>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'COMx))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'COMy))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'COMz))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Swing_x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Swing_y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'Swing_z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'leg)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <traj>) istream)
  "Deserializes a message object of type '<traj>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'COMx) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'COMy) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'COMz) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Swing_x) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Swing_y) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Swing_z) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'leg) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<traj>)))
  "Returns string type for a message object of type '<traj>"
  "leg_ik/traj")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'traj)))
  "Returns string type for a message object of type 'traj"
  "leg_ik/traj")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<traj>)))
  "Returns md5sum for a message object of type '<traj>"
  "5ec2ca23b0e126f2f1c1263a9a369c4b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'traj)))
  "Returns md5sum for a message object of type 'traj"
  "5ec2ca23b0e126f2f1c1263a9a369c4b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<traj>)))
  "Returns full string definition for message of type '<traj>"
  (cl:format cl:nil "float64 COMx~%float64 COMy~%float64 COMz~%float64 Swing_x~%float64 Swing_y~%float64 Swing_z~%int32   leg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'traj)))
  "Returns full string definition for message of type 'traj"
  (cl:format cl:nil "float64 COMx~%float64 COMy~%float64 COMz~%float64 Swing_x~%float64 Swing_y~%float64 Swing_z~%int32   leg~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <traj>))
  (cl:+ 0
     8
     8
     8
     8
     8
     8
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <traj>))
  "Converts a ROS message object to a list"
  (cl:list 'traj
    (cl:cons ':COMx (COMx msg))
    (cl:cons ':COMy (COMy msg))
    (cl:cons ':COMz (COMz msg))
    (cl:cons ':Swing_x (Swing_x msg))
    (cl:cons ':Swing_y (Swing_y msg))
    (cl:cons ':Swing_z (Swing_z msg))
    (cl:cons ':leg (leg msg))
))
