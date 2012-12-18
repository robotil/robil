
(cl:in-package :asdf)

(defsystem "C11_Agent-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "C11C32_PATH" :depends-on ("_package_C11C32_PATH"))
    (:file "_package_C11C32_PATH" :depends-on ("_package"))
    (:file "C32C11_PATH" :depends-on ("_package_C32C11_PATH"))
    (:file "_package_C32C11_PATH" :depends-on ("_package"))
    (:file "C34C11_STT" :depends-on ("_package_C34C11_STT"))
    (:file "_package_C34C11_STT" :depends-on ("_package"))
  ))