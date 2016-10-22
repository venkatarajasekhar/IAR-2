
(cl:in-package :asdf)

(defsystem "subsomption-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Channel" :depends-on ("_package_Channel"))
    (:file "_package_Channel" :depends-on ("_package"))
  ))