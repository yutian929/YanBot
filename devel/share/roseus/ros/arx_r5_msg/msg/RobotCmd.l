;; Auto-generated. Do not edit!


(when (boundp 'arx_r5_msg::RobotCmd)
  (if (not (find-package "ARX_R5_MSG"))
    (make-package "ARX_R5_MSG"))
  (shadow 'RobotCmd (find-package "ARX_R5_MSG")))
(unless (find-package "ARX_R5_MSG::ROBOTCMD")
  (make-package "ARX_R5_MSG::ROBOTCMD"))

(in-package "ROS")
;;//! \htmlinclude RobotCmd.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass arx_r5_msg::RobotCmd
  :super ros::object
  :slots (_header _end_pos _joint_pos _gripper _mode ))

(defmethod arx_r5_msg::RobotCmd
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:end_pos __end_pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:joint_pos __joint_pos) (make-array 6 :initial-element 0.0 :element-type :float))
    ((:gripper __gripper) 0.0)
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _header __header)
   (setq _end_pos __end_pos)
   (setq _joint_pos __joint_pos)
   (setq _gripper (float __gripper))
   (setq _mode (round __mode))
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
  (:gripper
   (&optional __gripper)
   (if __gripper (setq _gripper __gripper)) _gripper)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; float64[6] _end_pos
    (* 8    6)
    ;; float64[6] _joint_pos
    (* 8    6)
    ;; float64 _gripper
    8
    ;; int64 _mode
    8
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
     ;; float64[6] _joint_pos
     (dotimes (i 6)
       (sys::poke (elt _joint_pos i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64 _gripper
       (sys::poke _gripper (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int64 _mode
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _mode (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _mode) (= (length (_mode . bv)) 2)) ;; bignum
              (write-long (ash (elt (_mode . bv) 0) 0) s)
              (write-long (ash (elt (_mode . bv) 1) -1) s))
             ((and (class _mode) (= (length (_mode . bv)) 1)) ;; big1
              (write-long (elt (_mode . bv) 0) s)
              (write-long (if (>= _mode 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _mode s)(write-long (if (>= _mode 0) 0 #xffffffff) s)))
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
   ;; float64[6] _joint_pos
   (dotimes (i (length _joint_pos))
     (setf (elt _joint_pos i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; float64 _gripper
     (setq _gripper (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int64 _mode
#+(or :alpha :irix6 :x86_64)
      (setf _mode (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _mode (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(setf (get arx_r5_msg::RobotCmd :md5sum-) "547e6ecc540b9df32417036b604fe3df")
(setf (get arx_r5_msg::RobotCmd :datatype-) "arx_r5_msg/RobotCmd")
(setf (get arx_r5_msg::RobotCmd :definition-)
      "std_msgs/Header header

float64[6] end_pos # x y z w x y z

float64[6] joint_pos

float64 gripper

int64 mode

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



(provide :arx_r5_msg/RobotCmd "547e6ecc540b9df32417036b604fe3df")


