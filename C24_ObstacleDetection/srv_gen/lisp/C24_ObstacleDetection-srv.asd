
(cl:in-package :asdf)

(defsystem "C24_ObstacleDetection-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C24_ObstacleDetection-msg
)
  :components ((:file "_package")
    (:file "C24" :depends-on ("_package_C24"))
    (:file "_package_C24" :depends-on ("_package"))
  ))