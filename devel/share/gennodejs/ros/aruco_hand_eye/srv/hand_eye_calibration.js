// Auto-generated. Do not edit!

// (in-package aruco_hand_eye.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------


//-----------------------------------------------------------

class hand_eye_calibrationRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.cmd = null;
      this.end_trans = null;
    }
    else {
      if (initObj.hasOwnProperty('cmd')) {
        this.cmd = initObj.cmd
      }
      else {
        this.cmd = '';
      }
      if (initObj.hasOwnProperty('end_trans')) {
        this.end_trans = initObj.end_trans
      }
      else {
        this.end_trans = new geometry_msgs.msg.Transform();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type hand_eye_calibrationRequest
    // Serialize message field [cmd]
    bufferOffset = _serializer.string(obj.cmd, buffer, bufferOffset);
    // Serialize message field [end_trans]
    bufferOffset = geometry_msgs.msg.Transform.serialize(obj.end_trans, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type hand_eye_calibrationRequest
    let len;
    let data = new hand_eye_calibrationRequest(null);
    // Deserialize message field [cmd]
    data.cmd = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [end_trans]
    data.end_trans = geometry_msgs.msg.Transform.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.cmd.length;
    return length + 60;
  }

  static datatype() {
    // Returns string type for a service object
    return 'aruco_hand_eye/hand_eye_calibrationRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3bd7467818137612ca9b122514057be8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string                  cmd
    geometry_msgs/Transform end_trans
    
    
    ================================================================================
    MSG: geometry_msgs/Transform
    # This represents the transform between two coordinate frames in free space.
    
    Vector3 translation
    Quaternion rotation
    
    ================================================================================
    MSG: geometry_msgs/Vector3
    # This represents a vector in free space. 
    # It is only meant to represent a direction. Therefore, it does not
    # make sense to apply a translation to it (e.g., when applying a 
    # generic rigid transformation to a Vector3, tf2 will only apply the
    # rotation). If you want your data to be translatable too, use the
    # geometry_msgs/Point message instead.
    
    float64 x
    float64 y
    float64 z
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new hand_eye_calibrationRequest(null);
    if (msg.cmd !== undefined) {
      resolved.cmd = msg.cmd;
    }
    else {
      resolved.cmd = ''
    }

    if (msg.end_trans !== undefined) {
      resolved.end_trans = geometry_msgs.msg.Transform.Resolve(msg.end_trans)
    }
    else {
      resolved.end_trans = new geometry_msgs.msg.Transform()
    }

    return resolved;
    }
};

class hand_eye_calibrationResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.is_done = null;
      this.end2cam_trans = null;
    }
    else {
      if (initObj.hasOwnProperty('is_done')) {
        this.is_done = initObj.is_done
      }
      else {
        this.is_done = false;
      }
      if (initObj.hasOwnProperty('end2cam_trans')) {
        this.end2cam_trans = initObj.end2cam_trans
      }
      else {
        this.end2cam_trans = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type hand_eye_calibrationResponse
    // Serialize message field [is_done]
    bufferOffset = _serializer.bool(obj.is_done, buffer, bufferOffset);
    // Serialize message field [end2cam_trans]
    bufferOffset = _arraySerializer.float64(obj.end2cam_trans, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type hand_eye_calibrationResponse
    let len;
    let data = new hand_eye_calibrationResponse(null);
    // Deserialize message field [is_done]
    data.is_done = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [end2cam_trans]
    data.end2cam_trans = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.end2cam_trans.length;
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'aruco_hand_eye/hand_eye_calibrationResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3c47a15d4529b441b5135e0cc3d6db0f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool                    is_done
    float64[]             end2cam_trans
    
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new hand_eye_calibrationResponse(null);
    if (msg.is_done !== undefined) {
      resolved.is_done = msg.is_done;
    }
    else {
      resolved.is_done = false
    }

    if (msg.end2cam_trans !== undefined) {
      resolved.end2cam_trans = msg.end2cam_trans;
    }
    else {
      resolved.end2cam_trans = []
    }

    return resolved;
    }
};

module.exports = {
  Request: hand_eye_calibrationRequest,
  Response: hand_eye_calibrationResponse,
  md5sum() { return 'a0517a9a02aee46cf83b64864c094a8b'; },
  datatype() { return 'aruco_hand_eye/hand_eye_calibration'; }
};
