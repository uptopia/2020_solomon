// Auto-generated. Do not edit!

// (in-package yolo_detection.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ROI = require('./ROI.js');

//-----------------------------------------------------------

class ROI_array {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ROI_list = null;
    }
    else {
      if (initObj.hasOwnProperty('ROI_list')) {
        this.ROI_list = initObj.ROI_list
      }
      else {
        this.ROI_list = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ROI_array
    // Serialize message field [ROI_list]
    // Serialize the length for message field [ROI_list]
    bufferOffset = _serializer.uint32(obj.ROI_list.length, buffer, bufferOffset);
    obj.ROI_list.forEach((val) => {
      bufferOffset = ROI.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ROI_array
    let len;
    let data = new ROI_array(null);
    // Deserialize message field [ROI_list]
    // Deserialize array length for message field [ROI_list]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.ROI_list = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.ROI_list[i] = ROI.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.ROI_list.forEach((val) => {
      length += ROI.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yolo_detection/ROI_array';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '98a75627170c780818a3f3f2d48f82c4';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ROI[] ROI_list
    
    ================================================================================
    MSG: yolo_detection/ROI
    string object_name
    int32 x
    int32 y
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ROI_array(null);
    if (msg.ROI_list !== undefined) {
      resolved.ROI_list = new Array(msg.ROI_list.length);
      for (let i = 0; i < resolved.ROI_list.length; ++i) {
        resolved.ROI_list[i] = ROI.Resolve(msg.ROI_list[i]);
      }
    }
    else {
      resolved.ROI_list = []
    }

    return resolved;
    }
};

module.exports = ROI_array;
