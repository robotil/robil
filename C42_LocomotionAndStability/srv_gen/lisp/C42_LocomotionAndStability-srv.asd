
(cl:in-package :asdf)

(defsystem "C42_LocomotionAndStability-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C42_LocomotionAndStability-msg
)
  :components ((:file "_package")
    (:file "C42" :depends-on ("_package_C42"))
    (:file "_package_C42" :depends-on ("_package"))
  ))