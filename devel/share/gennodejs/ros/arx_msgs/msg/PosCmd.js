// Auto-generated. Do not edit!

// (in-package arx_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class PosCmd {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.x = null;
      this.y = null;
      this.z = null;
      this.roll = null;
      this.pitch = null;
      this.yaw = null;
      this.gripper = null;
      this.quater_x = null;
      this.quater_y = null;
      this.quater_z = null;
      this.quater_w = null;
      this.chx = null;
      this.chy = null;
      this.chz = null;
      this.vel_l = null;
      this.vel_r = null;
      this.height = null;
      this.head_pit = null;
      this.head_yaw = null;
      this.temp_float_data = null;
      this.temp_int_data = null;
      this.mode1 = null;
      this.mode2 = null;
      this.time_count = null;
    }
    else {
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = 0.0;
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = 0.0;
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = 0.0;
      }
      if (initObj.hasOwnProperty('roll')) {
        this.roll = initObj.roll
      }
      else {
        this.roll = 0.0;
      }
      if (initObj.hasOwnProperty('pitch')) {
        this.pitch = initObj.pitch
      }
      else {
        this.pitch = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('gripper')) {
        this.gripper = initObj.gripper
      }
      else {
        this.gripper = 0.0;
      }
      if (initObj.hasOwnProperty('quater_x')) {
        this.quater_x = initObj.quater_x
      }
      else {
        this.quater_x = 0.0;
      }
      if (initObj.hasOwnProperty('quater_y')) {
        this.quater_y = initObj.quater_y
      }
      else {
        this.quater_y = 0.0;
      }
      if (initObj.hasOwnProperty('quater_z')) {
        this.quater_z = initObj.quater_z
      }
      else {
        this.quater_z = 0.0;
      }
      if (initObj.hasOwnProperty('quater_w')) {
        this.quater_w = initObj.quater_w
      }
      else {
        this.quater_w = 0.0;
      }
      if (initObj.hasOwnProperty('chx')) {
        this.chx = initObj.chx
      }
      else {
        this.chx = 0.0;
      }
      if (initObj.hasOwnProperty('chy')) {
        this.chy = initObj.chy
      }
      else {
        this.chy = 0.0;
      }
      if (initObj.hasOwnProperty('chz')) {
        this.chz = initObj.chz
      }
      else {
        this.chz = 0.0;
      }
      if (initObj.hasOwnProperty('vel_l')) {
        this.vel_l = initObj.vel_l
      }
      else {
        this.vel_l = 0.0;
      }
      if (initObj.hasOwnProperty('vel_r')) {
        this.vel_r = initObj.vel_r
      }
      else {
        this.vel_r = 0.0;
      }
      if (initObj.hasOwnProperty('height')) {
        this.height = initObj.height
      }
      else {
        this.height = 0.0;
      }
      if (initObj.hasOwnProperty('head_pit')) {
        this.head_pit = initObj.head_pit
      }
      else {
        this.head_pit = 0.0;
      }
      if (initObj.hasOwnProperty('head_yaw')) {
        this.head_yaw = initObj.head_yaw
      }
      else {
        this.head_yaw = 0.0;
      }
      if (initObj.hasOwnProperty('temp_float_data')) {
        this.temp_float_data = initObj.temp_float_data
      }
      else {
        this.temp_float_data = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('temp_int_data')) {
        this.temp_int_data = initObj.temp_int_data
      }
      else {
        this.temp_int_data = new Array(6).fill(0);
      }
      if (initObj.hasOwnProperty('mode1')) {
        this.mode1 = initObj.mode1
      }
      else {
        this.mode1 = 0;
      }
      if (initObj.hasOwnProperty('mode2')) {
        this.mode2 = initObj.mode2
      }
      else {
        this.mode2 = 0;
      }
      if (initObj.hasOwnProperty('time_count')) {
        this.time_count = initObj.time_count
      }
      else {
        this.time_count = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type PosCmd
    // Serialize message field [x]
    bufferOffset = _serializer.float64(obj.x, buffer, bufferOffset);
    // Serialize message field [y]
    bufferOffset = _serializer.float64(obj.y, buffer, bufferOffset);
    // Serialize message field [z]
    bufferOffset = _serializer.float64(obj.z, buffer, bufferOffset);
    // Serialize message field [roll]
    bufferOffset = _serializer.float64(obj.roll, buffer, bufferOffset);
    // Serialize message field [pitch]
    bufferOffset = _serializer.float64(obj.pitch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [gripper]
    bufferOffset = _serializer.float64(obj.gripper, buffer, bufferOffset);
    // Serialize message field [quater_x]
    bufferOffset = _serializer.float64(obj.quater_x, buffer, bufferOffset);
    // Serialize message field [quater_y]
    bufferOffset = _serializer.float64(obj.quater_y, buffer, bufferOffset);
    // Serialize message field [quater_z]
    bufferOffset = _serializer.float64(obj.quater_z, buffer, bufferOffset);
    // Serialize message field [quater_w]
    bufferOffset = _serializer.float64(obj.quater_w, buffer, bufferOffset);
    // Serialize message field [chx]
    bufferOffset = _serializer.float64(obj.chx, buffer, bufferOffset);
    // Serialize message field [chy]
    bufferOffset = _serializer.float64(obj.chy, buffer, bufferOffset);
    // Serialize message field [chz]
    bufferOffset = _serializer.float64(obj.chz, buffer, bufferOffset);
    // Serialize message field [vel_l]
    bufferOffset = _serializer.float64(obj.vel_l, buffer, bufferOffset);
    // Serialize message field [vel_r]
    bufferOffset = _serializer.float64(obj.vel_r, buffer, bufferOffset);
    // Serialize message field [height]
    bufferOffset = _serializer.float64(obj.height, buffer, bufferOffset);
    // Serialize message field [head_pit]
    bufferOffset = _serializer.float64(obj.head_pit, buffer, bufferOffset);
    // Serialize message field [head_yaw]
    bufferOffset = _serializer.float64(obj.head_yaw, buffer, bufferOffset);
    // Check that the constant length array field [temp_float_data] has the right length
    if (obj.temp_float_data.length !== 6) {
      throw new Error('Unable to serialize array field temp_float_data - length must be 6')
    }
    // Serialize message field [temp_float_data]
    bufferOffset = _arraySerializer.float64(obj.temp_float_data, buffer, bufferOffset, 6);
    // Check that the constant length array field [temp_int_data] has the right length
    if (obj.temp_int_data.length !== 6) {
      throw new Error('Unable to serialize array field temp_int_data - length must be 6')
    }
    // Serialize message field [temp_int_data]
    bufferOffset = _arraySerializer.int32(obj.temp_int_data, buffer, bufferOffset, 6);
    // Serialize message field [mode1]
    bufferOffset = _serializer.int32(obj.mode1, buffer, bufferOffset);
    // Serialize message field [mode2]
    bufferOffset = _serializer.int32(obj.mode2, buffer, bufferOffset);
    // Serialize message field [time_count]
    bufferOffset = _serializer.int32(obj.time_count, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type PosCmd
    let len;
    let data = new PosCmd(null);
    // Deserialize message field [x]
    data.x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [y]
    data.y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [z]
    data.z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [roll]
    data.roll = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pitch]
    data.pitch = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [gripper]
    data.gripper = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [quater_x]
    data.quater_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [quater_y]
    data.quater_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [quater_z]
    data.quater_z = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [quater_w]
    data.quater_w = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [chx]
    data.chx = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [chy]
    data.chy = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [chz]
    data.chz = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_l]
    data.vel_l = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [vel_r]
    data.vel_r = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [height]
    data.height = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [head_pit]
    data.head_pit = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [head_yaw]
    data.head_yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [temp_float_data]
    data.temp_float_data = _arrayDeserializer.float64(buffer, bufferOffset, 6)
    // Deserialize message field [temp_int_data]
    data.temp_int_data = _arrayDeserializer.int32(buffer, bufferOffset, 6)
    // Deserialize message field [mode1]
    data.mode1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mode2]
    data.mode2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [time_count]
    data.time_count = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 236;
  }

  static datatype() {
    // Returns string type for a message object
    return 'arx_msgs/PosCmd';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8199bc204509c58732480f82fd3dfaed';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 x
    float64 y
    float64 z
    float64 roll
    float64 pitch
    float64 yaw
    float64 gripper
    float64 quater_x
    float64 quater_y
    float64 quater_z
    float64 quater_w
    float64 chx
    float64 chy
    float64 chz
    float64 vel_l
    float64 vel_r
    float64 height
    float64 head_pit
    float64 head_yaw
    float64[6] temp_float_data
    int32[6] temp_int_data
    int32 mode1
    int32 mode2
    int32 time_count
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new PosCmd(null);
    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = 0.0
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = 0.0
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = 0.0
    }

    if (msg.roll !== undefined) {
      resolved.roll = msg.roll;
    }
    else {
      resolved.roll = 0.0
    }

    if (msg.pitch !== undefined) {
      resolved.pitch = msg.pitch;
    }
    else {
      resolved.pitch = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.gripper !== undefined) {
      resolved.gripper = msg.gripper;
    }
    else {
      resolved.gripper = 0.0
    }

    if (msg.quater_x !== undefined) {
      resolved.quater_x = msg.quater_x;
    }
    else {
      resolved.quater_x = 0.0
    }

    if (msg.quater_y !== undefined) {
      resolved.quater_y = msg.quater_y;
    }
    else {
      resolved.quater_y = 0.0
    }

    if (msg.quater_z !== undefined) {
      resolved.quater_z = msg.quater_z;
    }
    else {
      resolved.quater_z = 0.0
    }

    if (msg.quater_w !== undefined) {
      resolved.quater_w = msg.quater_w;
    }
    else {
      resolved.quater_w = 0.0
    }

    if (msg.chx !== undefined) {
      resolved.chx = msg.chx;
    }
    else {
      resolved.chx = 0.0
    }

    if (msg.chy !== undefined) {
      resolved.chy = msg.chy;
    }
    else {
      resolved.chy = 0.0
    }

    if (msg.chz !== undefined) {
      resolved.chz = msg.chz;
    }
    else {
      resolved.chz = 0.0
    }

    if (msg.vel_l !== undefined) {
      resolved.vel_l = msg.vel_l;
    }
    else {
      resolved.vel_l = 0.0
    }

    if (msg.vel_r !== undefined) {
      resolved.vel_r = msg.vel_r;
    }
    else {
      resolved.vel_r = 0.0
    }

    if (msg.height !== undefined) {
      resolved.height = msg.height;
    }
    else {
      resolved.height = 0.0
    }

    if (msg.head_pit !== undefined) {
      resolved.head_pit = msg.head_pit;
    }
    else {
      resolved.head_pit = 0.0
    }

    if (msg.head_yaw !== undefined) {
      resolved.head_yaw = msg.head_yaw;
    }
    else {
      resolved.head_yaw = 0.0
    }

    if (msg.temp_float_data !== undefined) {
      resolved.temp_float_data = msg.temp_float_data;
    }
    else {
      resolved.temp_float_data = new Array(6).fill(0)
    }

    if (msg.temp_int_data !== undefined) {
      resolved.temp_int_data = msg.temp_int_data;
    }
    else {
      resolved.temp_int_data = new Array(6).fill(0)
    }

    if (msg.mode1 !== undefined) {
      resolved.mode1 = msg.mode1;
    }
    else {
      resolved.mode1 = 0
    }

    if (msg.mode2 !== undefined) {
      resolved.mode2 = msg.mode2;
    }
    else {
      resolved.mode2 = 0
    }

    if (msg.time_count !== undefined) {
      resolved.time_count = msg.time_count;
    }
    else {
      resolved.time_count = 0
    }

    return resolved;
    }
};

module.exports = PosCmd;
