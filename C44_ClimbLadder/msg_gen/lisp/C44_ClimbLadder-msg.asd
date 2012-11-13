
(cl:in-package :asdf)

(defsystem "C44_ClimbLadder-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "C23C44_DIS" :depends-on ("_package_C23C44_DIS"))
    (:file "_package_C23C44_DIS" :depends-on ("_package"))
    (:file "C44C0_CLS" :depends-on ("_package_C44C0_CLS"))
    (:file "_package_C44C0_CLS" :depends-on ("_package"))
    (:file "C23C44_LDIM" :depends-on ("_package_C23C44_LDIM"))
    (:file "_package_C23C44_LDIM" :depends-on ("_package"))
  ))