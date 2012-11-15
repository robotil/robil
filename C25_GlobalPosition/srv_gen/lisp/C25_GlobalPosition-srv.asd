
(cl:in-package :asdf)

(defsystem "C25_GlobalPosition-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :C25_GlobalPosition-msg
)
  :components ((:file "_package")
    (:file "C25" :depends-on ("_package_C25"))
    (:file "_package_C25" :depends-on ("_package"))
  ))