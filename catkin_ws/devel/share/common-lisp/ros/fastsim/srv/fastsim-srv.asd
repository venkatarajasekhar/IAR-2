
(cl:in-package :asdf)

(defsystem "fastsim-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Teleport" :depends-on ("_package_Teleport"))
    (:file "_package_Teleport" :depends-on ("_package"))
    (:file "UpdateDisplay" :depends-on ("_package_UpdateDisplay"))
    (:file "_package_UpdateDisplay" :depends-on ("_package"))
  ))