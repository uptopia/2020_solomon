// Auto-generated. Do not edit!

// (in-package aruco_hand_eye.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class aruco_infoRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.id = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = '';
      }
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type aruco_infoRequest
    // Serialize message field [cmd]
    bufferOffset = _serializer.string(obj.cmd, buffer, bufferOffset);
    // Serialize message field [id]
    bufferOffset = _serializer.int64(obj.id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type aruco_infoRequest
    let len;
    let data = new aruco_infoRequest(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [id]
    data.id = _deserializer.int64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.cmd.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'aruco_hand_eye/aruco_infoRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5e4ce8b3f259efb471169b58c08d8e1e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string    cmd
    int64   id
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new aruco_infoRequest(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = ''
    }

    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    return resolved;
    }
};

class aruco_infoResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ids = null;
      this.tvecs = null;
      this.rvecs = null;
    }
    else {
      if (initObj.hasOwnProperty('ids')) {
        this.ids = initObj.ids
      }
      else {
        this.ids = [];
      }
      if (initObj.hasOwnProperty('tvecs')) {
        this.tvecs = initObj.tvecs
      }
      else {
        this.tvecs = [];
      }
      if (initObj.hasOwnProperty('rvecs')) {
        this.rvecs = initObj.rvecs
      }
      else {
        this.rvecs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type aruco_infoResponse
    // Serialize message field [ids]
    bufferOffset = _arraySerializer.int64(obj.ids, buffer, bufferOffset, null);
    // Serialize message field [tvecs]
    bufferOffset = _arraySerializer.float64(obj.tvecs, buffer, bufferOffset, null);
    // Serialize message field [rvecs]
    bufferOffset = _arraySerializer.float64(obj.rvecs, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type aruco_infoResponse
    let len;
    let data = new aruco_infoResponse(null);
    // Deserialize message field [ids]
    data.ids = _arrayDeserializer.int64(buffer, bufferOffset, null)
    // Deserialize message field [tvecs]
    data.tvecs = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [rvecs]
    data.rvecs = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.ids.length;
    length += 8 * object.tvecs.length;
    length += 8 * object.rvecs.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'aruco_hand_eye/aruco_infoResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c86bfd11a5f6ab9f79643988c45acbb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    
    int64[]   ids
    float64[] tvecs
    float64[] rvecs
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new aruco_infoResponse(null);
    if (msg.ids !== undefined) {
      resolved.ids = msg.ids;
    }
    else {
      resolved.ids = []
    }

    if (msg.tvecs !== undefined) {
      resolved.tvecs = msg.tvecs;
    }
    else {
      resolved.tvecs = []
    }

    if (msg.rvecs !== undefined) {
      resolved.rvecs = msg.rvecs;
    }
    else {
      resolved.rvecs = []
    }

    return resolved;
    }
};

module.exports = {
  Request: aruco_infoRequest,
  Response: aruco_infoResponse,
  md5sum() { return '1bc8d5c8b1ad95abb0b2391cce4ff505'; },
  datatype() { return 'aruco_hand_eye/aruco_info'; }
};
