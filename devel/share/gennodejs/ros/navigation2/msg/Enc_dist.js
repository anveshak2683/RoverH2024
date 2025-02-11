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

class Enc_dist {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dist = null;
    }
    else {
      if (initObj.hasOwnProperty('dist')) {
        this.dist = initObj.dist
      }
      else {
        this.dist = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Enc_dist
    // Serialize message field [dist]
    bufferOffset = _serializer.float64(obj.dist, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Enc_dist
    let len;
    let data = new Enc_dist(null);
    // Deserialize message field [dist]
    data.dist = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navigation2/Enc_dist';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3f4fece6412db25720b2bab9f80b3473';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 dist
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Enc_dist(null);
    if (msg.dist !== undefined) {
      resolved.dist = msg.dist;
    }
    else {
      resolved.dist = 0.0
    }

    return resolved;
    }
};

module.exports = Enc_dist;
