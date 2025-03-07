;; Auto-generated. Do not edit!


(when (boundp 'arx_r5_msg::RobotStatus)
  (if (not (find-package "ARX_R5_MSG"))
    (make-package "ARX_R5_MSG"))
  (shadow 'RobotStatus (find-package "ARX_R5_MSG")))
(unless (find-package "ARX_R5_MSG::ROBOTSTATUS")
  (make-package "ARX_R5_MSG::ROBOTSTATUS"))

(in-package "ROS")
;;//! \htmlinclude RobotStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arx_r5_msg::RobotStatus
  :super ros::object
  :slots (_header _end_pos _joint_pos _joint_vel _joint_cur ))

(defmethod arx_r5_msg::RobotStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:end_pos __end_pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_pos __joint_pos) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:joint_vel __joint_vel) (make-array 7 :initial-element 0.0 :element-type :float))
    ((:joint_cur __joint_cur) (make-array 7 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _header __header)
   (setq _end_pos __end_pos)
   (setq _joint_pos __joint_pos)
   (setq _joint_vel __joint_vel)
   (setq _joint_cur __joint_cur)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:end_pos
   (&optional __end_pos)
   (if __end_pos (setq _end_pos __end_pos)) _end_pos)
  (:joint_pos
   (&optional __joint_pos)
   (if __joint_pos (setq _joint_pos __joint_pos)) _joint_pos)
  (:joint_vel
   (&optional __joint_vel)
   (if __joint_vel (setq _joint_vel __joint_vel)) _joint_vel)
  (:joint_cur
   (&optional __joint_cur)
   (if __joint_cur (setq _joint_cur __joint_cur)) _joint_cur)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[6] _end_pos
    (* 8    6)
    ;; float64[7] _joint_pos
    (* 8    7)
    ;; float64[7] _joint_vel
    (* 8    7)
    ;; float64[7] _joint_cur
    (* 8    7)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; float64[6] _end_pos
     (dotimes (i 6)
       (sys::poke (elt _end_pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _joint_pos
     (dotimes (i 7)
       (sys::poke (elt _joint_pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _joint_vel
     (dotimes (i 7)
       (sys::poke (elt _joint_vel i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[7] _joint_cur
     (dotimes (i 7)
       (sys::poke (elt _joint_cur i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; float64[6] _end_pos
   (dotimes (i (length _end_pos))
     (setf (elt _end_pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _joint_pos
   (dotimes (i (length _joint_pos))
     (setf (elt _joint_pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _joint_vel
   (dotimes (i (length _joint_vel))
     (setf (elt _joint_vel i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64[7] _joint_cur
   (dotimes (i (length _joint_cur))
     (setf (elt _joint_cur i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;;
   self)
  )

(setf (get arx_r5_msg::RobotStatus :md5sum-) "6ad8a32d4f4533dc88ed7ddb658fce71")
(setf (get arx_r5_msg::RobotStatus :datatype-) "arx_r5_msg/RobotStatus")
(setf (get arx_r5_msg::RobotStatus :definition-)
      "std_msgs/Header header

float64[6] end_pos

float64[7] joint_pos
float64[7] joint_vel
float64[7] joint_cur

================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

")



(provide :arx_r5_msg/RobotStatus "6ad8a32d4f4533dc88ed7ddb658fce71")


