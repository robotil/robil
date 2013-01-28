
(cl:in-package :asdf)

(defsystem "leg_ik-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "LegAngle" :depends-on ("_package_LegAngle"))
    (:file "_package_LegAngle" :depends-on ("_package"))
    (:file "traj" :depends-on ("_package_traj"))
    (:file "_package_traj" :depends-on ("_package"))
  ))