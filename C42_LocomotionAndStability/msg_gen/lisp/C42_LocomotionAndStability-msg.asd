
(cl:in-package :asdf)

(defsystem "C42_LocomotionAndStability-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "C42C34_CS" :depends-on ("_package_C42C34_CS"))
    (:file "_package_C42C34_CS" :depends-on ("_package"))
    (:file "C34C42_WM" :depends-on ("_package_C34C42_WM"))
    (:file "_package_C34C42_WM" :depends-on ("_package"))
    (:file "C34C42_PSU" :depends-on ("_package_C34C42_PSU"))
    (:file "_package_C34C42_PSU" :depends-on ("_package"))
    (:file "C42C34_EVE" :depends-on ("_package_C42C34_EVE"))
    (:file "_package_C42C34_EVE" :depends-on ("_package"))
  ))