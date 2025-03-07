// Auto-generated. Do not edit!

// (in-package grounding_sam_ros.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let sensor_msgs = _finder('sensor_msgs');

//-----------------------------------------------------------

let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class VitDetectionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.color_image = null;
      this.prompt = null;
    }
    else {
      if (initObj.hasOwnProperty('color_image')) {
        this.color_image = initObj.color_image
      }
      else {
        this.color_image = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('prompt')) {
        this.prompt = initObj.prompt
      }
      else {
        this.prompt = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VitDetectionRequest
    // Serialize message field [color_image]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.color_image, buffer, bufferOffset);
    // Serialize message field [prompt]
    bufferOffset = _serializer.string(obj.prompt, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VitDetectionRequest
    let len;
    let data = new VitDetectionRequest(null);
    // Deserialize message field [color_image]
    data.color_image = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [prompt]
    data.prompt = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += sensor_msgs.msg.Image.getMessageSize(object.color_image);
    length += _getByteLength(object.prompt);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'grounding_sam_ros/VitDetectionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b4f89486a1428e7fd375da4293bd8185';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # request params
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VitDetectionRequest(null);
    if (msg.color_image !== undefined) {
      resolved.color_image = sensor_msgs.msg.Image.Resolve(msg.color_image)
    }
    else {
      resolved.color_image = new sensor_msgs.msg.Image()
    }

    if (msg.prompt !== undefined) {
      resolved.prompt = msg.prompt;
    }
    else {
      resolved.prompt = ''
    }

    return resolved;
    }
};

class VitDetectionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.labels = null;
      this.class_id = null;
      this.boxes = null;
      this.scores = null;
      this.annotated_frame = null;
      this.segmasks = null;
    }
    else {
      if (initObj.hasOwnProperty('labels')) {
        this.labels = initObj.labels
      }
      else {
        this.labels = [];
      }
      if (initObj.hasOwnProperty('class_id')) {
        this.class_id = initObj.class_id
      }
      else {
        this.class_id = [];
      }
      if (initObj.hasOwnProperty('boxes')) {
        this.boxes = initObj.boxes
      }
      else {
        this.boxes = new std_msgs.msg.Float32MultiArray();
      }
      if (initObj.hasOwnProperty('scores')) {
        this.scores = initObj.scores
      }
      else {
        this.scores = [];
      }
      if (initObj.hasOwnProperty('annotated_frame')) {
        this.annotated_frame = initObj.annotated_frame
      }
      else {
        this.annotated_frame = new sensor_msgs.msg.Image();
      }
      if (initObj.hasOwnProperty('segmasks')) {
        this.segmasks = initObj.segmasks
      }
      else {
        this.segmasks = new std_msgs.msg.Float32MultiArray();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VitDetectionResponse
    // Serialize message field [labels]
    bufferOffset = _arraySerializer.string(obj.labels, buffer, bufferOffset, null);
    // Serialize message field [class_id]
    bufferOffset = _arraySerializer.int32(obj.class_id, buffer, bufferOffset, null);
    // Serialize message field [boxes]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.boxes, buffer, bufferOffset);
    // Serialize message field [scores]
    bufferOffset = _arraySerializer.float32(obj.scores, buffer, bufferOffset, null);
    // Serialize message field [annotated_frame]
    bufferOffset = sensor_msgs.msg.Image.serialize(obj.annotated_frame, buffer, bufferOffset);
    // Serialize message field [segmasks]
    bufferOffset = std_msgs.msg.Float32MultiArray.serialize(obj.segmasks, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VitDetectionResponse
    let len;
    let data = new VitDetectionResponse(null);
    // Deserialize message field [labels]
    data.labels = _arrayDeserializer.string(buffer, bufferOffset, null)
    // Deserialize message field [class_id]
    data.class_id = _arrayDeserializer.int32(buffer, bufferOffset, null)
    // Deserialize message field [boxes]
    data.boxes = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    // Deserialize message field [scores]
    data.scores = _arrayDeserializer.float32(buffer, bufferOffset, null)
    // Deserialize message field [annotated_frame]
    data.annotated_frame = sensor_msgs.msg.Image.deserialize(buffer, bufferOffset);
    // Deserialize message field [segmasks]
    data.segmasks = std_msgs.msg.Float32MultiArray.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.labels.forEach((val) => {
      length += 4 + _getByteLength(val);
    });
    length += 4 * object.class_id.length;
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.boxes);
    length += 4 * object.scores.length;
    length += sensor_msgs.msg.Image.getMessageSize(object.annotated_frame);
    length += std_msgs.msg.Float32MultiArray.getMessageSize(object.segmasks);
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'grounding_sam_ros/VitDetectionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc9fa720d67167700993761350e404df';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    # dim[0].label  = "height"
    # dim[0].size   = 480
    # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
    # dim[1].label  = "width"
    # dim[1].size   = 640
    # dim[1].stride = 3*640 = 1920
    # dim[2].label  = "channel"
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VitDetectionResponse(null);
    if (msg.labels !== undefined) {
      resolved.labels = msg.labels;
    }
    else {
      resolved.labels = []
    }

    if (msg.class_id !== undefined) {
      resolved.class_id = msg.class_id;
    }
    else {
      resolved.class_id = []
    }

    if (msg.boxes !== undefined) {
      resolved.boxes = std_msgs.msg.Float32MultiArray.Resolve(msg.boxes)
    }
    else {
      resolved.boxes = new std_msgs.msg.Float32MultiArray()
    }

    if (msg.scores !== undefined) {
      resolved.scores = msg.scores;
    }
    else {
      resolved.scores = []
    }

    if (msg.annotated_frame !== undefined) {
      resolved.annotated_frame = sensor_msgs.msg.Image.Resolve(msg.annotated_frame)
    }
    else {
      resolved.annotated_frame = new sensor_msgs.msg.Image()
    }

    if (msg.segmasks !== undefined) {
      resolved.segmasks = std_msgs.msg.Float32MultiArray.Resolve(msg.segmasks)
    }
    else {
      resolved.segmasks = new std_msgs.msg.Float32MultiArray()
    }

    return resolved;
    }
};

module.exports = {
  Request: VitDetectionRequest,
  Response: VitDetectionResponse,
  md5sum() { return '66a90572757b1f3f2629d2e4bea82842'; },
  datatype() { return 'grounding_sam_ros/VitDetection'; }
};
