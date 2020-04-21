
(cl:in-package :asdf)

(defsystem "mrm_training_force-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "Floatarray" :depends-on ("_package_Floatarray"))
    (:file "_package_Floatarray" :depends-on ("_package"))
  ))