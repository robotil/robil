
(cl:in-package :asdf)

(defsystem "C43_LocalBodyPVA-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C43_LocalBodyPVA-msg
)
  :components ((:file "_package")
    (:file "C43" :depends-on ("_package_C43"))
    (:file "_package_C43" :depends-on ("_package"))
  ))