
(cl:in-package :asdf)

(defsystem "C41_QuasiStaticWalking-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "QuasiStaticWalkingActionResult" :depends-on ("_package_QuasiStaticWalkingActionResult"))
    (:file "_package_QuasiStaticWalkingActionResult" :depends-on ("_package"))
    (:file "QuasiStaticWalkingResult" :depends-on ("_package_QuasiStaticWalkingResult"))
    (:file "_package_QuasiStaticWalkingResult" :depends-on ("_package"))
    (:file "QuasiStaticWalkingActionGoal" :depends-on ("_package_QuasiStaticWalkingActionGoal"))
    (:file "_package_QuasiStaticWalkingActionGoal" :depends-on ("_package"))
    (:file "QuasiStaticWalkingFeedback" :depends-on ("_package_QuasiStaticWalkingFeedback"))
    (:file "_package_QuasiStaticWalkingFeedback" :depends-on ("_package"))
    (:file "QuasiStaticWalkingActionFeedback" :depends-on ("_package_QuasiStaticWalkingActionFeedback"))
    (:file "_package_QuasiStaticWalkingActionFeedback" :depends-on ("_package"))
    (:file "QuasiStaticWalkingAction" :depends-on ("_package_QuasiStaticWalkingAction"))
    (:file "_package_QuasiStaticWalkingAction" :depends-on ("_package"))
    (:file "QuasiStaticWalkingGoal" :depends-on ("_package_QuasiStaticWalkingGoal"))
    (:file "_package_QuasiStaticWalkingGoal" :depends-on ("_package"))
  ))