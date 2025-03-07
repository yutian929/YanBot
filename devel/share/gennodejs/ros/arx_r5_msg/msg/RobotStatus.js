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

class RobotStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.end_pos = null;
      this.joint_pos = null;
      this.joint_vel = null;
      this.joint_cur = null;
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
        this.joint_pos = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('joint_vel')) {
        this.joint_vel = initObj.joint_vel
      }
      else {
        this.joint_vel = new Array(7).fill(0);
      }
      if (initObj.hasOwnProperty('joint_cur')) {
        this.joint_cur = initObj.joint_cur
      }
      else {
        this.joint_cur = new Array(7).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RobotStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Check that the constant length array field [end_pos] has the right length
    if (obj.end_pos.length !== 6) {
      throw new Error('Unable to serialize array field end_pos - length must be 6')
    }
    // Serialize message field [end_pos]
    bufferOffset = _arraySerializer.float64(obj.end_pos, buffer, bufferOffset, 6);
    // Check that the constant length array field [joint_pos] has the right length
    if (obj.joint_pos.length !== 7) {
      throw new Error('Unable to serialize array field joint_pos - length must be 7')
    }
    // Serialize message field [joint_pos]
    bufferOffset = _arraySerializer.float64(obj.joint_pos, buffer, bufferOffset, 7);
    // Check that the constant length array field [joint_vel] has the right length
    if (obj.joint_vel.length !== 7) {
      throw new Error('Unable to serialize array field joint_vel - length must be 7')
    }
    // Serialize message field [joint_vel]
    bufferOffset = _arraySerializer.float64(obj.joint_vel, buffer, bufferOffset, 7);
    // Check that the constant length array field [joint_cur] has the right length
    if (obj.joint_cur.length !== 7) {
      throw new Error('Unable to serialize array field joint_cur - length must be 7')
    }
    // Serialize message field [joint_cur]
    bufferOffset = _arraySerializer.float64(obj.joint_cur, buffer, bufferOffset, 7);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RobotStatus
    let len;
    let data = new RobotStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [end_pos]
    data.end_pos = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [joint_pos]
    data.joint_pos = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [joint_vel]
    data.joint_vel = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    // Deserialize message field [joint_cur]
    data.joint_cur = _arrayDeserializer.float64(buffer, bufferOffset, 7)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    return length + 216;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arx_r5_msg/RobotStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6ad8a32d4f4533dc88ed7ddb658fce71';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RobotStatus(null);
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
      resolved.joint_pos = new Array(7).fill(0)
    }

    if (msg.joint_vel !== undefined) {
      resolved.joint_vel = msg.joint_vel;
    }
    else {
      resolved.joint_vel = new Array(7).fill(0)
    }

    if (msg.joint_cur !== undefined) {
      resolved.joint_cur = msg.joint_cur;
    }
    else {
      resolved.joint_cur = new Array(7).fill(0)
    }

    return resolved;
    }
};

module.exports = RobotStatus;
