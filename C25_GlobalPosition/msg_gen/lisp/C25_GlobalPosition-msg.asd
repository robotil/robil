
(cl:in-package :asdf)

(defsystem "C25_GlobalPosition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :nav_msgs-msg
               :sensor_msgs-msg
)
  :components ((:file "_package")
    (:file "C25C0_OPO" :depends-on ("_package_C25C0_OPO"))
    (:file "_package_C25C0_OPO" :depends-on ("_package"))
    (:file "C25C0_ROP" :depends-on ("_package_C25C0_ROP"))
    (:file "_package_C25C0_ROP" :depends-on ("_package"))
    (:file "C0C25_AZI" :depends-on ("_package_C0C25_AZI"))
    (:file "_package_C0C25_AZI" :depends-on ("_package"))
    (:file "C0C25_CAM" :depends-on ("_package_C0C25_CAM"))
    (:file "_package_C0C25_CAM" :depends-on ("_package"))
    (:file "C0C25_LAZ" :depends-on ("_package_C0C25_LAZ"))
    (:file "_package_C0C25_LAZ" :depends-on ("_package"))
    (:file "UTM" :depends-on ("_package_UTM"))
    (:file "_package_UTM" :depends-on ("_package"))
  ))