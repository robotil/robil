
(cl:in-package :asdf)

(defsystem "C11_Agent-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C11_Agent-msg
)
  :components ((:file "_package")
    (:file "C11" :depends-on ("_package_C11"))
    (:file "_package_C11" :depends-on ("_package"))
  ))