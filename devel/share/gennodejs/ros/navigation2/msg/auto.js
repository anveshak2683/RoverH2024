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

class auto {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.arm = null;
      this.latitude = null;
      this.longitude = null;
      this.setstage = null;
      this.text = null;
      this.aruco_coordinates = null;
      this.reference = null;
      this.msg_id = null;
    }
    else {
      if (initObj.hasOwnProperty('arm')) {
        this.arm = initObj.arm
      }
      else {
        this.arm = false;
      }
      if (initObj.hasOwnProperty('latitude')) {
        this.latitude = initObj.latitude
      }
      else {
        this.latitude = 0.0;
      }
      if (initObj.hasOwnProperty('longitude')) {
        this.longitude = initObj.longitude
      }
      else {
        this.longitude = 0.0;
      }
      if (initObj.hasOwnProperty('setstage')) {
        this.setstage = initObj.setstage
      }
      else {
        this.setstage = 0;
      }
      if (initObj.hasOwnProperty('text')) {
        this.text = initObj.text
      }
      else {
        this.text = '';
      }
      if (initObj.hasOwnProperty('aruco_coordinates')) {
        this.aruco_coordinates = initObj.aruco_coordinates
      }
      else {
        this.aruco_coordinates = [];
      }
      if (initObj.hasOwnProperty('reference')) {
        this.reference = initObj.reference
      }
      else {
        this.reference = '';
      }
      if (initObj.hasOwnProperty('msg_id')) {
        this.msg_id = initObj.msg_id
      }
      else {
        this.msg_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type auto
    // Serialize message field [arm]
    bufferOffset = _serializer.bool(obj.arm, buffer, bufferOffset);
    // Serialize message field [latitude]
    bufferOffset = _serializer.float64(obj.latitude, buffer, bufferOffset);
    // Serialize message field [longitude]
    bufferOffset = _serializer.float64(obj.longitude, buffer, bufferOffset);
    // Serialize message field [setstage]
    bufferOffset = _serializer.int8(obj.setstage, buffer, bufferOffset);
    // Serialize message field [text]
    bufferOffset = _serializer.string(obj.text, buffer, bufferOffset);
    // Serialize message field [aruco_coordinates]
    bufferOffset = _arraySerializer.float64(obj.aruco_coordinates, buffer, bufferOffset, null);
    // Serialize message field [reference]
    bufferOffset = _serializer.string(obj.reference, buffer, bufferOffset);
    // Serialize message field [msg_id]
    bufferOffset = _serializer.int8(obj.msg_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type auto
    let len;
    let data = new auto(null);
    // Deserialize message field [arm]
    data.arm = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [latitude]
    data.latitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [longitude]
    data.longitude = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [setstage]
    data.setstage = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [text]
    data.text = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [aruco_coordinates]
    data.aruco_coordinates = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [reference]
    data.reference = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [msg_id]
    data.msg_id = _deserializer.int8(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.text);
    length += 8 * object.aruco_coordinates.length;
    length += _getByteLength(object.reference);
    return length + 31;
  }

  static datatype() {
    // Returns string type for a message object
    return 'navigation2/auto';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '65672b1bd9673047ebc0672e3417ec0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool arm
    float64 latitude 
    float64 longitude
    int8 setstage 
    string text
    float64[] aruco_coordinates
    string reference 
    int8 msg_id
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new auto(null);
    if (msg.arm !== undefined) {
      resolved.arm = msg.arm;
    }
    else {
      resolved.arm = false
    }

    if (msg.latitude !== undefined) {
      resolved.latitude = msg.latitude;
    }
    else {
      resolved.latitude = 0.0
    }

    if (msg.longitude !== undefined) {
      resolved.longitude = msg.longitude;
    }
    else {
      resolved.longitude = 0.0
    }

    if (msg.setstage !== undefined) {
      resolved.setstage = msg.setstage;
    }
    else {
      resolved.setstage = 0
    }

    if (msg.text !== undefined) {
      resolved.text = msg.text;
    }
    else {
      resolved.text = ''
    }

    if (msg.aruco_coordinates !== undefined) {
      resolved.aruco_coordinates = msg.aruco_coordinates;
    }
    else {
      resolved.aruco_coordinates = []
    }

    if (msg.reference !== undefined) {
      resolved.reference = msg.reference;
    }
    else {
      resolved.reference = ''
    }

    if (msg.msg_id !== undefined) {
      resolved.msg_id = msg.msg_id;
    }
    else {
      resolved.msg_id = 0
    }

    return resolved;
    }
};

module.exports = auto;
