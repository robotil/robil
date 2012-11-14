
(cl:in-package :asdf)

(defsystem "C11_OperatorControl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "C11" :depends-on ("_package_C11"))
    (:file "_package_C11" :depends-on ("_package"))
  ))