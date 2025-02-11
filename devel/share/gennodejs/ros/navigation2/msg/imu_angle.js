// Auto-generated. Do not edit!

// (in-package navigation2.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class imu_angle {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.Roll = null;
      this.Pitch = null;
      this.Yaw = null;
    }
    else {
      if (initObj.hasOwnProperty('Roll')) {
        this.Roll = initObj.Roll
      }
      else {
        this.Roll = 0.0;
      }
      if (initObj.hasOwnProperty('Pitch')) {
        this.Pitch = initObj.Pitch
      }
      else {
        this.Pitch = 0.0;
      }
      if (initObj.hasOwnProperty('Yaw')) {
        this.Yaw = initObj.Yaw
      }
      else {
        this.Yaw = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type imu_angle
    // Serialize message field [Roll]
    bufferOffset = _serializer.float32(obj.Roll, buffer, bufferOffset);
    // Serialize message field [Pitch]
    bufferOffset = _serializer.float32(obj.Pitch, buffer, bufferOffset);
    // Serialize message field [Yaw]
    bufferOffset = _serializer.float32(obj.Yaw, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type imu_angle
    let len;
    let data = new imu_angle(null);
    // Deserialize message field [Roll]
    data.Roll = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Pitch]
    data.Pitch = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [Yaw]
    data.Yaw = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navigation2/imu_angle';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd8aec502c207f580f351d8f6036c0af0';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 Roll
    float32 Pitch
    float32 Yaw
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new imu_angle(null);
    if (msg.Roll !== undefined) {
      resolved.Roll = msg.Roll;
    }
    else {
      resolved.Roll = 0.0
    }

    if (msg.Pitch !== undefined) {
      resolved.Pitch = msg.Pitch;
    }
    else {
      resolved.Pitch = 0.0
    }

    if (msg.Yaw !== undefined) {
      resolved.Yaw = msg.Yaw;
    }
    else {
      resolved.Yaw = 0.0
    }

    return resolved;
    }
};

module.exports = imu_angle;
