
(cl:in-package :asdf)

(defsystem "C23_ObjectRecognition-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TBD" :depends-on ("_package_TBD"))
    (:file "_package_TBD" :depends-on ("_package"))
    (:file "C23C0_OPO" :depends-on ("_package_C23C0_OPO"))
    (:file "_package_C23C0_OPO" :depends-on ("_package"))
    (:file "C23C0_OD" :depends-on ("_package_C23C0_OD"))
    (:file "_package_C23C0_OD" :depends-on ("_package"))
    (:file "C0C23_SEC" :depends-on ("_package_C0C23_SEC"))
    (:file "_package_C0C23_SEC" :depends-on ("_package"))
    (:file "C0C23_SAR" :depends-on ("_package_C0C23_SAR"))
    (:file "_package_C0C23_SAR" :depends-on ("_package"))
    (:file "C23C0_ODIM" :depends-on ("_package_C23C0_ODIM"))
    (:file "_package_C23C0_ODIM" :depends-on ("_package"))
    (:file "C0C23_SEOB" :depends-on ("_package_C0C23_SEOB"))
    (:file "_package_C0C23_SEOB" :depends-on ("_package"))
  ))