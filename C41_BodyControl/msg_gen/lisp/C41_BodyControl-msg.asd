
(cl:in-package :asdf)

(defsystem "C41_BodyControl-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "C0C41_PVA" :depends-on ("_package_C0C41_PVA"))
    (:file "_package_C0C41_PVA" :depends-on ("_package"))
    (:file "C0C41_WM" :depends-on ("_package_C0C41_WM"))
    (:file "_package_C0C41_WM" :depends-on ("_package"))
    (:file "C41C0_APVA" :depends-on ("_package_C41C0_APVA"))
    (:file "_package_C41C0_APVA" :depends-on ("_package"))
    (:file "C0C41_TC" :depends-on ("_package_C0C41_TC"))
    (:file "_package_C0C41_TC" :depends-on ("_package"))
    (:file "C0C41_LOAD" :depends-on ("_package_C0C41_LOAD"))
    (:file "_package_C0C41_LOAD" :depends-on ("_package"))
    (:file "C41C0_AT" :depends-on ("_package_C41C0_AT"))
    (:file "_package_C41C0_AT" :depends-on ("_package"))
  ))