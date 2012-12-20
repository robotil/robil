; Auto-generated. Do not edit!


(cl:in-package C66_Grasp-msg)


;//! \htmlinclude C66_GraspFeedback.msg.html

(cl:defclass <C66_GraspFeedback> (roslisp-msg-protocol:ros-message)
  ((finger0
    :reader finger0
    :initarg :finger0
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (finger1
    :reader finger1
    :initarg :finger1
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (finger2
    :reader finger2
    :initarg :finger2
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0))
   (finger3
    :reader finger3
    :initarg :finger3
    :type (cl:vector cl:float)
   :initform (cl:make-array 3 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass C66_GraspFeedback (<C66_GraspFeedback>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <C66_GraspFeedback>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'C66_GraspFeedback)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name C66_Grasp-msg:<C66_GraspFeedback> is deprecated: use C66_Grasp-msg:C66_GraspFeedback instead.")))

(cl:ensure-generic-function 'finger0-val :lambda-list '(m))
(cl:defmethod finger0-val ((m <C66_GraspFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C66_Grasp-msg:finger0-val is deprecated.  Use C66_Grasp-msg:finger0 instead.")
  (finger0 m))

(cl:ensure-generic-function 'finger1-val :lambda-list '(m))
(cl:defmethod finger1-val ((m <C66_GraspFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C66_Grasp-msg:finger1-val is deprecated.  Use C66_Grasp-msg:finger1 instead.")
  (finger1 m))

(cl:ensure-generic-function 'finger2-val :lambda-list '(m))
(cl:defmethod finger2-val ((m <C66_GraspFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C66_Grasp-msg:finger2-val is deprecated.  Use C66_Grasp-msg:finger2 instead.")
  (finger2 m))

(cl:ensure-generic-function 'finger3-val :lambda-list '(m))
(cl:defmethod finger3-val ((m <C66_GraspFeedback>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader C66_Grasp-msg:finger3-val is deprecated.  Use C66_Grasp-msg:finger3 instead.")
  (finger3 m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <C66_GraspFeedback>) ostream)
  "Serializes a message object of type '<C66_GraspFeedback>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'finger0))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'finger1))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'finger2))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'finger3))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <C66_GraspFeedback>) istream)
  "Deserializes a message object of type '<C66_GraspFeedback>"
  (cl:setf (cl:slot-value msg 'finger0) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'finger0)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'finger1) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'finger1)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'finger2) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'finger2)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (cl:setf (cl:slot-value msg 'finger3) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'finger3)))
    (cl:dotimes (i 3)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<C66_GraspFeedback>)))
  "Returns string type for a message object of type '<C66_GraspFeedback>"
  "C66_Grasp/C66_GraspFeedback")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'C66_GraspFeedback)))
  "Returns string type for a message object of type 'C66_GraspFeedback"
  "C66_Grasp/C66_GraspFeedback")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<C66_GraspFeedback>)))
  "Returns md5sum for a message object of type '<C66_GraspFeedback>"
  "0c3e18fddf61e2fdd223a61162ef1076")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'C66_GraspFeedback)))
  "Returns md5sum for a message object of type 'C66_GraspFeedback"
  "0c3e18fddf61e2fdd223a61162ef1076")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<C66_GraspFeedback>)))
  "Returns full string definition for message of type '<C66_GraspFeedback>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float32[3] finger0~%float32[3] finger1~%float32[3] finger2~%float32[3] finger3~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'C66_GraspFeedback)))
  "Returns full string definition for message of type 'C66_GraspFeedback"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback~%float32[3] finger0~%float32[3] finger1~%float32[3] finger2~%float32[3] finger3~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <C66_GraspFeedback>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'finger0) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'finger1) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'finger2) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'finger3) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <C66_GraspFeedback>))
  "Converts a ROS message object to a list"
  (cl:list 'C66_GraspFeedback
    (cl:cons ':finger0 (finger0 msg))
    (cl:cons ':finger1 (finger1 msg))
    (cl:cons ':finger2 (finger2 msg))
    (cl:cons ':finger3 (finger3 msg))
))
