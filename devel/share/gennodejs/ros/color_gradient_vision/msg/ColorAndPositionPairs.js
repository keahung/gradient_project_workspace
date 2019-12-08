// Auto-generated. Do not edit!

// (in-package color_gradient_vision.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ColorAndPosition = require('./ColorAndPosition.js');

//-----------------------------------------------------------

class ColorAndPositionPairs {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.pairs = null;
    }
    else {
      if (initObj.hasOwnProperty('pairs')) {
        this.pairs = initObj.pairs
      }
      else {
        this.pairs = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ColorAndPositionPairs
    // Serialize message field [pairs]
    // Serialize the length for message field [pairs]
    bufferOffset = _serializer.uint32(obj.pairs.length, buffer, bufferOffset);
    obj.pairs.forEach((val) => {
      bufferOffset = ColorAndPosition.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ColorAndPositionPairs
    let len;
    let data = new ColorAndPositionPairs(null);
    // Deserialize message field [pairs]
    // Deserialize array length for message field [pairs]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.pairs = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.pairs[i] = ColorAndPosition.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 11 * object.pairs.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'color_gradient_vision/ColorAndPositionPairs';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'cc8e80026229df02835845177e306a2f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    ColorAndPosition[] pairs
    
    ================================================================================
    MSG: color_gradient_vision/ColorAndPosition
    float32 x
    float32 y
    uint8 R
    uint8 G
    uint8 B
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ColorAndPositionPairs(null);
    if (msg.pairs !== undefined) {
      resolved.pairs = new Array(msg.pairs.length);
      for (let i = 0; i < resolved.pairs.length; ++i) {
        resolved.pairs[i] = ColorAndPosition.Resolve(msg.pairs[i]);
      }
    }
    else {
      resolved.pairs = []
    }

    return resolved;
    }
};

module.exports = ColorAndPositionPairs;
