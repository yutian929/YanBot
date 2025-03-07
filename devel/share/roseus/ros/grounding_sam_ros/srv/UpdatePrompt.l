;; Auto-generated. Do not edit!


(when (boundp 'grounding_sam_ros::UpdatePrompt)
  (if (not (find-package "GROUNDING_SAM_ROS"))
    (make-package "GROUNDING_SAM_ROS"))
  (shadow 'UpdatePrompt (find-package "GROUNDING_SAM_ROS")))
(unless (find-package "GROUNDING_SAM_ROS::UPDATEPROMPT")
  (make-package "GROUNDING_SAM_ROS::UPDATEPROMPT"))
(unless (find-package "GROUNDING_SAM_ROS::UPDATEPROMPTREQUEST")
  (make-package "GROUNDING_SAM_ROS::UPDATEPROMPTREQUEST"))
(unless (find-package "GROUNDING_SAM_ROS::UPDATEPROMPTRESPONSE")
  (make-package "GROUNDING_SAM_ROS::UPDATEPROMPTRESPONSE"))

(in-package "ROS")





(defclass grounding_sam_ros::UpdatePromptRequest
  :super ros::object
  :slots (_data ))

(defmethod grounding_sam_ros::UpdatePromptRequest
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass grounding_sam_ros::UpdatePromptResponse
  :super ros::object
  :slots (_success _message ))

(defmethod grounding_sam_ros::UpdatePromptResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass grounding_sam_ros::UpdatePrompt
  :super ros::object
  :slots ())

(setf (get grounding_sam_ros::UpdatePrompt :md5sum-) "546971982e3fbbd5a41e60fb6432e357")
(setf (get grounding_sam_ros::UpdatePrompt :datatype-) "grounding_sam_ros/UpdatePrompt")
(setf (get grounding_sam_ros::UpdatePrompt :request) grounding_sam_ros::UpdatePromptRequest)
(setf (get grounding_sam_ros::UpdatePrompt :response) grounding_sam_ros::UpdatePromptResponse)

(defmethod grounding_sam_ros::UpdatePromptRequest
  (:response () (instance grounding_sam_ros::UpdatePromptResponse :init)))

(setf (get grounding_sam_ros::UpdatePromptRequest :md5sum-) "546971982e3fbbd5a41e60fb6432e357")
(setf (get grounding_sam_ros::UpdatePromptRequest :datatype-) "grounding_sam_ros/UpdatePromptRequest")
(setf (get grounding_sam_ros::UpdatePromptRequest :definition-)
      "# request params
string data
---
# response params
bool success
string message
")

(setf (get grounding_sam_ros::UpdatePromptResponse :md5sum-) "546971982e3fbbd5a41e60fb6432e357")
(setf (get grounding_sam_ros::UpdatePromptResponse :datatype-) "grounding_sam_ros/UpdatePromptResponse")
(setf (get grounding_sam_ros::UpdatePromptResponse :definition-)
      "# request params
string data
---
# response params
bool success
string message
")



(provide :grounding_sam_ros/UpdatePrompt "546971982e3fbbd5a41e60fb6432e357")


