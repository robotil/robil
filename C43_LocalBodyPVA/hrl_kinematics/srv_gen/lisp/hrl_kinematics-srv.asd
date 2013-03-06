
(cl:in-package :asdf)

(defsystem "hrl_kinematics-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SupportLegs_Status" :depends-on ("_package_SupportLegs_Status"))
    (:file "_package_SupportLegs_Status" :depends-on ("_package"))
  ))