
(cl:in-package :asdf)

(defsystem "C31_PathPlanner-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C31_PathPlanner-msg
)
  :components ((:file "_package")
    (:file "PathPlan" :depends-on ("_package_PathPlan"))
    (:file "_package_PathPlan" :depends-on ("_package"))
  ))