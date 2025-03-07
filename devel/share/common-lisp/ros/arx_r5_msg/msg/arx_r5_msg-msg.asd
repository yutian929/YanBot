
(cl:in-package :asdf)

(defsystem "arx_r5_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "RobotCmd" :depends-on ("_package_RobotCmd"))
    (:file "_package_RobotCmd" :depends-on ("_package"))
    (:file "RobotStatus" :depends-on ("_package_RobotStatus"))
    (:file "_package_RobotStatus" :depends-on ("_package"))
  ))