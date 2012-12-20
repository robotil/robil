
(cl:in-package :asdf)

(defsystem "C51_CarOperation-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C51_CarOperation-msg
)
  :components ((:file "_package")
    (:file "C51" :depends-on ("_package_C51"))
    (:file "_package_C51" :depends-on ("_package"))
  ))