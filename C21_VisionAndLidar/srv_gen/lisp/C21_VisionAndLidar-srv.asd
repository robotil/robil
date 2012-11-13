
(cl:in-package :asdf)

(defsystem "C21_VisionAndLidar-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C21_VisionAndLidar-msg
)
  :components ((:file "_package")
    (:file "C21" :depends-on ("_package_C21"))
    (:file "_package_C21" :depends-on ("_package"))
  ))