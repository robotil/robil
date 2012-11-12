
(cl:in-package :asdf)

(defsystem "C22_GroundRecognitionAndMapping-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C22_GroundRecognitionAndMapping-msg
)
  :components ((:file "_package")
    (:file "C22" :depends-on ("_package_C22"))
    (:file "_package_C22" :depends-on ("_package"))
  ))