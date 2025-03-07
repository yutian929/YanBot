;; Auto-generated. Do not edit!


(when (boundp 'grounding_sam_ros::VitDetection)
  (if (not (find-package "GROUNDING_SAM_ROS"))
    (make-package "GROUNDING_SAM_ROS"))
  (shadow 'VitDetection (find-package "GROUNDING_SAM_ROS")))
(unless (find-package "GROUNDING_SAM_ROS::VITDETECTION")
  (make-package "GROUNDING_SAM_ROS::VITDETECTION"))
(unless (find-package "GROUNDING_SAM_ROS::VITDETECTIONREQUEST")
  (make-package "GROUNDING_SAM_ROS::VITDETECTIONREQUEST"))
(unless (find-package "GROUNDING_SAM_ROS::VITDETECTIONRESPONSE")
  (make-package "GROUNDING_SAM_ROS::VITDETECTIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))


(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass grounding_sam_ros::VitDetectionRequest
  :super ros::object
  :slots (_color_image _prompt ))

(defmethod grounding_sam_ros::VitDetectionRequest
  (:init
   (&key
    ((:color_image __color_image) (instance sensor_msgs::Image :init))
    ((:prompt __prompt) "")
    )
   (send-super :init)
   (setq _color_image __color_image)
   (setq _prompt (string __prompt))
   self)
  (:color_image
   (&rest __color_image)
   (if (keywordp (car __color_image))
       (send* _color_image __color_image)
     (progn
       (if __color_image (setq _color_image (car __color_image)))
       _color_image)))
  (:prompt
   (&optional __prompt)
   (if __prompt (setq _prompt __prompt)) _prompt)
  (:serialization-length
   ()
   (+
    ;; sensor_msgs/Image _color_image
    (send _color_image :serialization-length)
    ;; string _prompt
    4 (length _prompt)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; sensor_msgs/Image _color_image
       (send _color_image :serialize s)
     ;; string _prompt
       (write-long (length _prompt) s) (princ _prompt s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; sensor_msgs/Image _color_image
     (send _color_image :deserialize buf ptr-) (incf ptr- (send _color_image :serialization-length))
   ;; string _prompt
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _prompt (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass grounding_sam_ros::VitDetectionResponse
  :super ros::object
  :slots (_labels _class_id _boxes _scores _annotated_frame _segmasks ))

(defmethod grounding_sam_ros::VitDetectionResponse
  (:init
   (&key
    ((:labels __labels) (let (r) (dotimes (i 0) (push "" r)) r))
    ((:class_id __class_id) (make-array 0 :initial-element 0 :element-type :integer))
    ((:boxes __boxes) (instance std_msgs::Float32MultiArray :init))
    ((:scores __scores) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:annotated_frame __annotated_frame) (instance sensor_msgs::Image :init))
    ((:segmasks __segmasks) (instance std_msgs::Float32MultiArray :init))
    )
   (send-super :init)
   (setq _labels __labels)
   (setq _class_id __class_id)
   (setq _boxes __boxes)
   (setq _scores __scores)
   (setq _annotated_frame __annotated_frame)
   (setq _segmasks __segmasks)
   self)
  (:labels
   (&optional __labels)
   (if __labels (setq _labels __labels)) _labels)
  (:class_id
   (&optional __class_id)
   (if __class_id (setq _class_id __class_id)) _class_id)
  (:boxes
   (&rest __boxes)
   (if (keywordp (car __boxes))
       (send* _boxes __boxes)
     (progn
       (if __boxes (setq _boxes (car __boxes)))
       _boxes)))
  (:scores
   (&optional __scores)
   (if __scores (setq _scores __scores)) _scores)
  (:annotated_frame
   (&rest __annotated_frame)
   (if (keywordp (car __annotated_frame))
       (send* _annotated_frame __annotated_frame)
     (progn
       (if __annotated_frame (setq _annotated_frame (car __annotated_frame)))
       _annotated_frame)))
  (:segmasks
   (&rest __segmasks)
   (if (keywordp (car __segmasks))
       (send* _segmasks __segmasks)
     (progn
       (if __segmasks (setq _segmasks (car __segmasks)))
       _segmasks)))
  (:serialization-length
   ()
   (+
    ;; string[] _labels
    (apply #'+ (mapcar #'(lambda (x) (+ 4 (length x))) _labels)) 4
    ;; int32[] _class_id
    (* 4    (length _class_id)) 4
    ;; std_msgs/Float32MultiArray _boxes
    (send _boxes :serialization-length)
    ;; float32[] _scores
    (* 4    (length _scores)) 4
    ;; sensor_msgs/Image _annotated_frame
    (send _annotated_frame :serialization-length)
    ;; std_msgs/Float32MultiArray _segmasks
    (send _segmasks :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string[] _labels
     (write-long (length _labels) s)
     (dolist (elem _labels)
       (write-long (length elem) s) (princ elem s)
       )
     ;; int32[] _class_id
     (write-long (length _class_id) s)
     (dotimes (i (length _class_id))
       (write-long (elt _class_id i) s)
       )
     ;; std_msgs/Float32MultiArray _boxes
       (send _boxes :serialize s)
     ;; float32[] _scores
     (write-long (length _scores) s)
     (dotimes (i (length _scores))
       (sys::poke (elt _scores i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; sensor_msgs/Image _annotated_frame
       (send _annotated_frame :serialize s)
     ;; std_msgs/Float32MultiArray _segmasks
       (send _segmasks :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string[] _labels
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _labels (make-list n))
     (dotimes (i n)
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setf (elt _labels i) (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
     ))
   ;; int32[] _class_id
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _class_id (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _class_id i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; std_msgs/Float32MultiArray _boxes
     (send _boxes :deserialize buf ptr-) (incf ptr- (send _boxes :serialization-length))
   ;; float32[] _scores
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _scores (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _scores i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     ))
   ;; sensor_msgs/Image _annotated_frame
     (send _annotated_frame :deserialize buf ptr-) (incf ptr- (send _annotated_frame :serialization-length))
   ;; std_msgs/Float32MultiArray _segmasks
     (send _segmasks :deserialize buf ptr-) (incf ptr- (send _segmasks :serialization-length))
   ;;
   self)
  )

(defclass grounding_sam_ros::VitDetection
  :super ros::object
  :slots ())

(setf (get grounding_sam_ros::VitDetection :md5sum-) "66a90572757b1f3f2629d2e4bea82842")
(setf (get grounding_sam_ros::VitDetection :datatype-) "grounding_sam_ros/VitDetection")
(setf (get grounding_sam_ros::VitDetection :request) grounding_sam_ros::VitDetectionRequest)
(setf (get grounding_sam_ros::VitDetection :response) grounding_sam_ros::VitDetectionResponse)

(defmethod grounding_sam_ros::VitDetectionRequest
  (:response () (instance grounding_sam_ros::VitDetectionResponse :init)))

(setf (get grounding_sam_ros::VitDetectionRequest :md5sum-) "66a90572757b1f3f2629d2e4bea82842")
(setf (get grounding_sam_ros::VitDetectionRequest :datatype-) "grounding_sam_ros/VitDetectionRequest")
(setf (get grounding_sam_ros::VitDetectionRequest :definition-)
      "# request params
sensor_msgs/Image color_image
string prompt

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
---
# response params
string[] labels
int32[] class_id
std_msgs/Float32MultiArray boxes
float32[] scores
sensor_msgs/Image annotated_frame
std_msgs/Float32MultiArray segmasks

================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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

(setf (get grounding_sam_ros::VitDetectionResponse :md5sum-) "66a90572757b1f3f2629d2e4bea82842")
(setf (get grounding_sam_ros::VitDetectionResponse :datatype-) "grounding_sam_ros/VitDetectionResponse")
(setf (get grounding_sam_ros::VitDetectionResponse :definition-)
      "# request params
sensor_msgs/Image color_image
string prompt

================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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
---
# response params
string[] labels
int32[] class_id
std_msgs/Float32MultiArray boxes
float32[] scores
sensor_msgs/Image annotated_frame
std_msgs/Float32MultiArray segmasks

================================================================================
MSG: std_msgs/Float32MultiArray
# Please look at the MultiArrayLayout message definition for
# documentation on all multiarrays.

MultiArrayLayout  layout        # specification of data layout
float32[]         data          # array of data


================================================================================
MSG: std_msgs/MultiArrayLayout
# The multiarray declares a generic multi-dimensional array of a
# particular data type.  Dimensions are ordered from outer most
# to inner most.

MultiArrayDimension[] dim # Array of dimension properties
uint32 data_offset        # padding elements at front of data

# Accessors should ALWAYS be written in terms of dimension stride
# and specified outer-most dimension first.
# 
# multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
#
# A standard, 3-channel 640x480 image with interleaved color channels
# would be specified as:
#
# dim[0].label  = \"height\"
# dim[0].size   = 480
# dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
# dim[1].label  = \"width\"
# dim[1].size   = 640
# dim[1].stride = 3*640 = 1920
# dim[2].label  = \"channel\"
# dim[2].size   = 3
# dim[2].stride = 3
#
# multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

================================================================================
MSG: std_msgs/MultiArrayDimension
string label   # label of given dimension
uint32 size    # size of given dimension (in type units)
uint32 stride  # stride of given dimension
================================================================================
MSG: sensor_msgs/Image
# This message contains an uncompressed image
# (0, 0) is at top-left corner of image
#

Header header        # Header timestamp should be acquisition time of image
                     # Header frame_id should be optical frame of camera
                     # origin of frame should be optical center of camera
                     # +x should point to the right in the image
                     # +y should point down in the image
                     # +z should point into to plane of the image
                     # If the frame_id here and the frame_id of the CameraInfo
                     # message associated with the image conflict
                     # the behavior is undefined

uint32 height         # image height, that is, number of rows
uint32 width          # image width, that is, number of columns

# The legal values for encoding are in file src/image_encodings.cpp
# If you want to standardize a new string format, join
# ros-users@lists.sourceforge.net and send an email proposing a new encoding.

string encoding       # Encoding of pixels -- channel meaning, ordering, size
                      # taken from the list of strings in include/sensor_msgs/image_encodings.h

uint8 is_bigendian    # is this data bigendian?
uint32 step           # Full row length in bytes
uint8[] data          # actual matrix data, size is (step * rows)

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



(provide :grounding_sam_ros/VitDetection "66a90572757b1f3f2629d2e4bea82842")


