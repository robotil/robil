
(cl:in-package :asdf)

(defsystem "C23_ObjectRecognition-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C23_ObjectRecognition-msg
)
  :components ((:file "_package")
    (:file "C23" :depends-on ("_package_C23"))
    (:file "_package_C23" :depends-on ("_package"))
  ))