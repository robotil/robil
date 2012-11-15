
(cl:in-package :asdf)

(defsystem "C44_ClimbLadder-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C44_ClimbLadder-msg
)
  :components ((:file "_package")
    (:file "C44" :depends-on ("_package_C44"))
    (:file "_package_C44" :depends-on ("_package"))
  ))