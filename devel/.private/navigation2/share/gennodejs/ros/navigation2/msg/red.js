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

class red {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel = null;
      this.omega = null;
      this.detect = null;
    }
    else {
      if (initObj.hasOwnProperty('vel')) {
        this.vel = initObj.vel
      }
      else {
        this.vel = 0;
      }
      if (initObj.hasOwnProperty('omega')) {
        this.omega = initObj.omega
      }
      else {
        this.omega = 0;
      }
      if (initObj.hasOwnProperty('detect')) {
        this.detect = initObj.detect
      }
      else {
        this.detect = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type red
    // Serialize message field [vel]
    bufferOffset = _serializer.int16(obj.vel, buffer, bufferOffset);
    // Serialize message field [omega]
    bufferOffset = _serializer.int16(obj.omega, buffer, bufferOffset);
    // Serialize message field [detect]
    bufferOffset = _serializer.bool(obj.detect, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type red
    let len;
    let data = new red(null);
    // Deserialize message field [vel]
    data.vel = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [omega]
    data.omega = _deserializer.int16(buffer, bufferOffset);
    // Deserialize message field [detect]
    data.detect = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navigation2/red';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '598737a047b4631c88d712e78758247d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int16 vel
    int16 omega
    bool detect
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new red(null);
    if (msg.vel !== undefined) {
      resolved.vel = msg.vel;
    }
    else {
      resolved.vel = 0
    }

    if (msg.omega !== undefined) {
      resolved.omega = msg.omega;
    }
    else {
      resolved.omega = 0
    }

    if (msg.detect !== undefined) {
      resolved.detect = msg.detect;
    }
    else {
      resolved.detect = false
    }

    return resolved;
    }
};

module.exports = red;
