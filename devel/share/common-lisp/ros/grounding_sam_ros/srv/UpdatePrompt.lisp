; Auto-generated. Do not edit!


(cl:in-package grounding_sam_ros-srv)


;//! \htmlinclude UpdatePrompt-request.msg.html

(cl:defclass <UpdatePrompt-request> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass UpdatePrompt-request (<UpdatePrompt-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdatePrompt-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdatePrompt-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name grounding_sam_ros-srv:<UpdatePrompt-request> is deprecated: use grounding_sam_ros-srv:UpdatePrompt-request instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <UpdatePrompt-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grounding_sam_ros-srv:data-val is deprecated.  Use grounding_sam_ros-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdatePrompt-request>) ostream)
  "Serializes a message object of type '<UpdatePrompt-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdatePrompt-request>) istream)
  "Deserializes a message object of type '<UpdatePrompt-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdatePrompt-request>)))
  "Returns string type for a service object of type '<UpdatePrompt-request>"
  "grounding_sam_ros/UpdatePromptRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdatePrompt-request)))
  "Returns string type for a service object of type 'UpdatePrompt-request"
  "grounding_sam_ros/UpdatePromptRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdatePrompt-request>)))
  "Returns md5sum for a message object of type '<UpdatePrompt-request>"
  "546971982e3fbbd5a41e60fb6432e357")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdatePrompt-request)))
  "Returns md5sum for a message object of type 'UpdatePrompt-request"
  "546971982e3fbbd5a41e60fb6432e357")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdatePrompt-request>)))
  "Returns full string definition for message of type '<UpdatePrompt-request>"
  (cl:format cl:nil "# request params~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdatePrompt-request)))
  "Returns full string definition for message of type 'UpdatePrompt-request"
  (cl:format cl:nil "# request params~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdatePrompt-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdatePrompt-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdatePrompt-request
    (cl:cons ':data (data msg))
))
;//! \htmlinclude UpdatePrompt-response.msg.html

(cl:defclass <UpdatePrompt-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass UpdatePrompt-response (<UpdatePrompt-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdatePrompt-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdatePrompt-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name grounding_sam_ros-srv:<UpdatePrompt-response> is deprecated: use grounding_sam_ros-srv:UpdatePrompt-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <UpdatePrompt-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grounding_sam_ros-srv:success-val is deprecated.  Use grounding_sam_ros-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <UpdatePrompt-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader grounding_sam_ros-srv:message-val is deprecated.  Use grounding_sam_ros-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdatePrompt-response>) ostream)
  "Serializes a message object of type '<UpdatePrompt-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdatePrompt-response>) istream)
  "Deserializes a message object of type '<UpdatePrompt-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdatePrompt-response>)))
  "Returns string type for a service object of type '<UpdatePrompt-response>"
  "grounding_sam_ros/UpdatePromptResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdatePrompt-response)))
  "Returns string type for a service object of type 'UpdatePrompt-response"
  "grounding_sam_ros/UpdatePromptResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdatePrompt-response>)))
  "Returns md5sum for a message object of type '<UpdatePrompt-response>"
  "546971982e3fbbd5a41e60fb6432e357")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdatePrompt-response)))
  "Returns md5sum for a message object of type 'UpdatePrompt-response"
  "546971982e3fbbd5a41e60fb6432e357")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdatePrompt-response>)))
  "Returns full string definition for message of type '<UpdatePrompt-response>"
  (cl:format cl:nil "# response params~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdatePrompt-response)))
  "Returns full string definition for message of type 'UpdatePrompt-response"
  (cl:format cl:nil "# response params~%bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdatePrompt-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdatePrompt-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdatePrompt-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdatePrompt)))
  'UpdatePrompt-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdatePrompt)))
  'UpdatePrompt-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdatePrompt)))
  "Returns string type for a service object of type '<UpdatePrompt>"
  "grounding_sam_ros/UpdatePrompt")