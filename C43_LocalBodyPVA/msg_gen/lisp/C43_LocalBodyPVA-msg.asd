
(cl:in-package :asdf)

(defsystem "C43_LocalBodyPVA-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "C43C0_JPVA" :depends-on ("_package_C43C0_JPVA"))
    (:file "_package_C43C0_JPVA" :depends-on ("_package"))
    (:file "C0C43_SJ" :depends-on ("_package_C0C43_SJ"))
    (:file "_package_C0C43_SJ" :depends-on ("_package"))
    (:file "C0C43_SL" :depends-on ("_package_C0C43_SL"))
    (:file "_package_C0C43_SL" :depends-on ("_package"))
    (:file "C43C0_LPVA" :depends-on ("_package_C43C0_LPVA"))
    (:file "_package_C43C0_LPVA" :depends-on ("_package"))
    (:file "C0C43_SI" :depends-on ("_package_C0C43_SI"))
    (:file "_package_C0C43_SI" :depends-on ("_package"))
    (:file "C0C43_SLI" :depends-on ("_package_C0C43_SLI"))
    (:file "_package_C0C43_SLI" :depends-on ("_package"))
  ))