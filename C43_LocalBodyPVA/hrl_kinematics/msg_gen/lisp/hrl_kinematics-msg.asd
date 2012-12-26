
(cl:in-package :asdf)

(defsystem "hrl_kinematics-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "pCoM_err_msg" :depends-on ("_package_pCoM_err_msg"))
    (:file "_package_pCoM_err_msg" :depends-on ("_package"))
    (:file "CoM_Array_msg" :depends-on ("_package_CoM_Array_msg"))
    (:file "_package_CoM_Array_msg" :depends-on ("_package"))
  ))