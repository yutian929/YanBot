// Auto-generated. Do not edit!

// (in-package arx_r5_msg.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RobotCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.end_pos = null;
      this.joint_pos = null;
      this.gripper = null;
      this.mode = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('end_pos')) {
        this.end_pos = initObj.end_pos
      }
      else {
        this.end_pos = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('joint_pos')) {
        this.joint_pos = initObj.joint_pos
      }
      else {
        this.joint_pos = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0.0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotCmd
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [end_pos] has the right length
    if (obj.end_pos.length !== 6) {
      throw new Error('Unable to serialize array field end_pos - length must be 6')
    }
    // Serialize message field [end_pos]
    bufferOffset = _arraySerializer.float64(obj.end_pos, buffer, bufferOffset, 6);
    // Check that the constant length array field [joint_pos] has the right length
    if (obj.joint_pos.length !== 6) {
      throw new Error('Unable to serialize array field joint_pos - length must be 6')
    }
    // Serialize message field [joint_pos]
    bufferOffset = _arraySerializer.float64(obj.joint_pos, buffer, bufferOffset, 6);
    // Serialize message field [gripper]
    bufferOffset = _serializer.float64(obj.gripper, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int64(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotCmd
    let len;
    let data = new RobotCmd(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_pos]
    data.end_pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [joint_pos]
    data.joint_pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [gripper]
    data.gripper = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 112;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arx_r5_msg/RobotCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '547e6ecc540b9df32417036b604fe3df';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotCmd(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.end_pos !== undefined) {
      resolved.end_pos = msg.end_pos;
    }
    else {
      resolved.end_pos = new Array(6).fill(0)
    }

    if (msg.joint_pos !== undefined) {
      resolved.joint_pos = msg.joint_pos;
    }
    else {
      resolved.joint_pos = new Array(6).fill(0)
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0.0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    return resolved;
    }
};

module.exports = RobotCmd;
