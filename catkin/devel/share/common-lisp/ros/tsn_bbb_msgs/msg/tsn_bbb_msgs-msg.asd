
(cl:in-package :asdf)

(defsystem "tsn_bbb_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Voltage" :depends-on ("_package_Voltage"))
    (:file "_package_Voltage" :depends-on ("_package"))
    (:file "GPIOOut" :depends-on ("_package_GPIOOut"))
    (:file "_package_GPIOOut" :depends-on ("_package"))
  ))