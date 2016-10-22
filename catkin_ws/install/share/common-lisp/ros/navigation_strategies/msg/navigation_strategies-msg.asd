
(cl:in-package :asdf)

(defsystem "navigation_strategies-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "DirDistrib" :depends-on ("_package_DirDistrib"))
    (:file "_package_DirDistrib" :depends-on ("_package"))
  ))