; Auto-generated. Do not edit!


(cl:in-package C43_LocalBodyPVA-msg)


;//! \htmlinclude C0C43_SLI.msg.html

(cl:defclass <C0C43_SLI> (roslisp-msg-protocol:ros-message)
  ((x_dir
    :reader x_dir
    :initarg :x_dir
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (y_dir
    :reader y_dir
    :initarg :y_dir
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (z_dir
    :reader z_dir
    :initarg :z_dir
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (yaw
    :reader yaw
    :initarg :yaw
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (pitch
    :reader pitch
    :initarg :pitch
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil))
   (roll
    :reader roll
    :initarg :roll
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 0 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass C0C43_SLI (<C0C43_SLI>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C0C43_SLI>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C0C43_SLI)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C43_LocalBodyPVA-msg:<C0C43_SLI> is deprecated: use C43_LocalBodyPVA-msg:C0C43_SLI instead.")))

(cl:ensure-generic-function 'x_dir-val :lambda-list '(m))
(cl:defmethod x_dir-val ((m <C0C43_SLI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:x_dir-val is deprecated.  Use C43_LocalBodyPVA-msg:x_dir instead.")
  (x_dir m))

(cl:ensure-generic-function 'y_dir-val :lambda-list '(m))
(cl:defmethod y_dir-val ((m <C0C43_SLI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:y_dir-val is deprecated.  Use C43_LocalBodyPVA-msg:y_dir instead.")
  (y_dir m))

(cl:ensure-generic-function 'z_dir-val :lambda-list '(m))
(cl:defmethod z_dir-val ((m <C0C43_SLI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:z_dir-val is deprecated.  Use C43_LocalBodyPVA-msg:z_dir instead.")
  (z_dir m))

(cl:ensure-generic-function 'yaw-val :lambda-list '(m))
(cl:defmethod yaw-val ((m <C0C43_SLI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:yaw-val is deprecated.  Use C43_LocalBodyPVA-msg:yaw instead.")
  (yaw m))

(cl:ensure-generic-function 'pitch-val :lambda-list '(m))
(cl:defmethod pitch-val ((m <C0C43_SLI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:pitch-val is deprecated.  Use C43_LocalBodyPVA-msg:pitch instead.")
  (pitch m))

(cl:ensure-generic-function 'roll-val :lambda-list '(m))
(cl:defmethod roll-val ((m <C0C43_SLI>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C43_LocalBodyPVA-msg:roll-val is deprecated.  Use C43_LocalBodyPVA-msg:roll instead.")
  (roll m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C0C43_SLI>) ostream)
  "Serializes a message object of type '<C0C43_SLI>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'x_dir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'x_dir))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'y_dir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'y_dir))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'z_dir))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'z_dir))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'yaw))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'pitch))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'roll))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C0C43_SLI>) istream)
  "Deserializes a message object of type '<C0C43_SLI>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'x_dir) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'x_dir)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'y_dir) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'y_dir)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'z_dir) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'z_dir)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'yaw) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'yaw)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pitch) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pitch)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'roll) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'roll)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C0C43_SLI>)))
  "Returns string type for a message object of type '<C0C43_SLI>"
  "C43_LocalBodyPVA/C0C43_SLI")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C0C43_SLI)))
  "Returns string type for a message object of type 'C0C43_SLI"
  "C43_LocalBodyPVA/C0C43_SLI")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C0C43_SLI>)))
  "Returns md5sum for a message object of type '<C0C43_SLI>"
  "0743aafc02bec381b4880fc30ed5d230")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C0C43_SLI)))
  "Returns md5sum for a message object of type 'C0C43_SLI"
  "0743aafc02bec381b4880fc30ed5d230")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C0C43_SLI>)))
  "Returns full string definition for message of type '<C0C43_SLI>"
  (cl:format cl:nil "bool[] x_dir~%bool[] y_dir~%bool[] z_dir~%bool[] yaw~%bool[] pitch~%bool[] roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C0C43_SLI)))
  "Returns full string definition for message of type 'C0C43_SLI"
  (cl:format cl:nil "bool[] x_dir~%bool[] y_dir~%bool[] z_dir~%bool[] yaw~%bool[] pitch~%bool[] roll~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C0C43_SLI>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'x_dir) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'y_dir) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'z_dir) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'yaw) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pitch) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'roll) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C0C43_SLI>))
  "Converts a ROS message object to a list"
  (cl:list 'C0C43_SLI
    (cl:cons ':x_dir (x_dir msg))
    (cl:cons ':y_dir (y_dir msg))
    (cl:cons ':z_dir (z_dir msg))
    (cl:cons ':yaw (yaw msg))
    (cl:cons ':pitch (pitch msg))
    (cl:cons ':roll (roll msg))
))
