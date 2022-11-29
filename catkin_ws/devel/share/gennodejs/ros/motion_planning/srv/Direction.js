// Auto-generated. Do not edit!

// (in-package motion_planning.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class DirectionRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.ready = null;
    }
    else {
      if (initObj.hasOwnProperty('ready')) {
        this.ready = initObj.ready
      }
      else {
        this.ready = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DirectionRequest
    // Serialize message field [ready]
    bufferOffset = _serializer.bool(obj.ready, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DirectionRequest
    let len;
    let data = new DirectionRequest(null);
    // Deserialize message field [ready]
    data.ready = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_planning/DirectionRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f378c6311f9e6ccd2cd8c5b327003f1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool ready
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DirectionRequest(null);
    if (msg.ready !== undefined) {
      resolved.ready = msg.ready;
    }
    else {
      resolved.ready = false
    }

    return resolved;
    }
};

class DirectionResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dir = null;
      this.ang = null;
      this.vec = null;
    }
    else {
      if (initObj.hasOwnProperty('dir')) {
        this.dir = initObj.dir
      }
      else {
        this.dir = '';
      }
      if (initObj.hasOwnProperty('ang')) {
        this.ang = initObj.ang
      }
      else {
        this.ang = 0.0;
      }
      if (initObj.hasOwnProperty('vec')) {
        this.vec = initObj.vec
      }
      else {
        this.vec = new Array(3).fill(0);
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type DirectionResponse
    // Serialize message field [dir]
    bufferOffset = _serializer.string(obj.dir, buffer, bufferOffset);
    // Serialize message field [ang]
    bufferOffset = _serializer.float32(obj.ang, buffer, bufferOffset);
    // Check that the constant length array field [vec] has the right length
    if (obj.vec.length !== 3) {
      throw new Error('Unable to serialize array field vec - length must be 3')
    }
    // Serialize message field [vec]
    bufferOffset = _arraySerializer.float32(obj.vec, buffer, bufferOffset, 3);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type DirectionResponse
    let len;
    let data = new DirectionResponse(null);
    // Deserialize message field [dir]
    data.dir = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [ang]
    data.ang = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [vec]
    data.vec = _arrayDeserializer.float32(buffer, bufferOffset, 3)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.dir);
    return length + 20;
  }

  static datatype() {
    // Returns string type for a service object
    return 'motion_planning/DirectionResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e9c03e485183aab1a8776292dfa81ccd';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string dir
    float32 ang
    float32[3] vec
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new DirectionResponse(null);
    if (msg.dir !== undefined) {
      resolved.dir = msg.dir;
    }
    else {
      resolved.dir = ''
    }

    if (msg.ang !== undefined) {
      resolved.ang = msg.ang;
    }
    else {
      resolved.ang = 0.0
    }

    if (msg.vec !== undefined) {
      resolved.vec = msg.vec;
    }
    else {
      resolved.vec = new Array(3).fill(0)
    }

    return resolved;
    }
};

module.exports = {
  Request: DirectionRequest,
  Response: DirectionResponse,
  md5sum() { return '91ac0cf00c7cda6ec66ed487aa4d6933'; },
  datatype() { return 'motion_planning/Direction'; }
};
