
(cl:in-package :asdf)

(defsystem "leg_ik-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :leg_ik-msg
)
  :components ((:file "_package")
    (:file "LegIk" :depends-on ("_package_LegIk"))
    (:file "_package_LegIk" :depends-on ("_package"))
    (:file "LegIkInit" :depends-on ("_package_LegIkInit"))
    (:file "_package_LegIkInit" :depends-on ("_package"))
  ))