// Auto-generated. Do not edit!

// (in-package aam_common_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Cone {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.position = null;
      this.color = null;
      this.ratioX = null;
      this.ratioY = null;
      this.pcX = null;
      this.pcY = null;
    }
    else {
      if (initObj.hasOwnProperty('position')) {
        this.position = initObj.position
      }
      else {
        this.position = new geometry_msgs.msg.Point();
      }
      if (initObj.hasOwnProperty('color')) {
        this.color = initObj.color
      }
      else {
        this.color = new std_msgs.msg.String();
      }
      if (initObj.hasOwnProperty('ratioX')) {
        this.ratioX = initObj.ratioX
      }
      else {
        this.ratioX = 0.0;
      }
      if (initObj.hasOwnProperty('ratioY')) {
        this.ratioY = initObj.ratioY
      }
      else {
        this.ratioY = 0.0;
      }
      if (initObj.hasOwnProperty('pcX')) {
        this.pcX = initObj.pcX
      }
      else {
        this.pcX = 0.0;
      }
      if (initObj.hasOwnProperty('pcY')) {
        this.pcY = initObj.pcY
      }
      else {
        this.pcY = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Cone
    // Serialize message field [position]
    bufferOffset = geometry_msgs.msg.Point.serialize(obj.position, buffer, bufferOffset);
    // Serialize message field [color]
    bufferOffset = std_msgs.msg.String.serialize(obj.color, buffer, bufferOffset);
    // Serialize message field [ratioX]
    bufferOffset = _serializer.float64(obj.ratioX, buffer, bufferOffset);
    // Serialize message field [ratioY]
    bufferOffset = _serializer.float64(obj.ratioY, buffer, bufferOffset);
    // Serialize message field [pcX]
    bufferOffset = _serializer.float64(obj.pcX, buffer, bufferOffset);
    // Serialize message field [pcY]
    bufferOffset = _serializer.float64(obj.pcY, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Cone
    let len;
    let data = new Cone(null);
    // Deserialize message field [position]
    data.position = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset);
    // Deserialize message field [color]
    data.color = std_msgs.msg.String.deserialize(buffer, bufferOffset);
    // Deserialize message field [ratioX]
    data.ratioX = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [ratioY]
    data.ratioY = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pcX]
    data.pcX = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pcY]
    data.pcY = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.String.getMessageSize(object.color);
    return length + 56;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aam_common_msgs/Cone';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0f501329c9e6188fb393747c57c79cb9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point position   # coordinate of cone in [x, y]
    std_msgs/String color   # color of cone, 'b' = blue, 'y' = yellow, 'so' = orange , 'bo' = orange
    float64 ratioX
    float64 ratioY
    float64 pcX
    float64 pcY
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: std_msgs/String
    string data
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Cone(null);
    if (msg.position !== undefined) {
      resolved.position = geometry_msgs.msg.Point.Resolve(msg.position)
    }
    else {
      resolved.position = new geometry_msgs.msg.Point()
    }

    if (msg.color !== undefined) {
      resolved.color = std_msgs.msg.String.Resolve(msg.color)
    }
    else {
      resolved.color = new std_msgs.msg.String()
    }

    if (msg.ratioX !== undefined) {
      resolved.ratioX = msg.ratioX;
    }
    else {
      resolved.ratioX = 0.0
    }

    if (msg.ratioY !== undefined) {
      resolved.ratioY = msg.ratioY;
    }
    else {
      resolved.ratioY = 0.0
    }

    if (msg.pcX !== undefined) {
      resolved.pcX = msg.pcX;
    }
    else {
      resolved.pcX = 0.0
    }

    if (msg.pcY !== undefined) {
      resolved.pcY = msg.pcY;
    }
    else {
      resolved.pcY = 0.0
    }

    return resolved;
    }
};

module.exports = Cone;
