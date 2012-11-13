
(cl:in-package :asdf)

(defsystem "C41_BodyControl-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C41_BodyControl-msg
)
  :components ((:file "_package")
    (:file "C41" :depends-on ("_package_C41"))
    (:file "_package_C41" :depends-on ("_package"))
  ))