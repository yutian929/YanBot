
(cl:in-package :asdf)

(defsystem "grounding_sam_ros-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :sensor_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "UpdatePrompt" :depends-on ("_package_UpdatePrompt"))
    (:file "_package_UpdatePrompt" :depends-on ("_package"))
    (:file "VitDetection" :depends-on ("_package_VitDetection"))
    (:file "_package_VitDetection" :depends-on ("_package"))
  ))