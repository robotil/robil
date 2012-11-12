
(cl:in-package :asdf)

(defsystem "C22_GroundRecognitionAndMapping-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "C0C22_CAM" :depends-on ("_package_C0C22_CAM"))
    (:file "_package_C0C22_CAM" :depends-on ("_package"))
    (:file "C0C22_SAF" :depends-on ("_package_C0C22_SAF"))
    (:file "_package_C0C22_SAF" :depends-on ("_package"))
    (:file "C0C22_AZI" :depends-on ("_package_C0C22_AZI"))
    (:file "_package_C0C22_AZI" :depends-on ("_package"))
    (:file "C22C0_PATH" :depends-on ("_package_C22C0_PATH"))
    (:file "_package_C22C0_PATH" :depends-on ("_package"))
    (:file "C0C22_LAZ" :depends-on ("_package_C0C22_LAZ"))
    (:file "_package_C0C22_LAZ" :depends-on ("_package"))
  ))