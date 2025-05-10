// Auto-generated. Do not edit!

// (in-package aam_common_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Cone = require('./Cone.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Map {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cone_yellow = null;
      this.cone_blue = null;
      this.orange = null;
      this.cone_unknow = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cone_yellow')) {
        this.cone_yellow = initObj.cone_yellow
      }
      else {
        this.cone_yellow = [];
      }
      if (initObj.hasOwnProperty('cone_blue')) {
        this.cone_blue = initObj.cone_blue
      }
      else {
        this.cone_blue = [];
      }
      if (initObj.hasOwnProperty('orange')) {
        this.orange = initObj.orange
      }
      else {
        this.orange = [];
      }
      if (initObj.hasOwnProperty('cone_unknow')) {
        this.cone_unknow = initObj.cone_unknow
      }
      else {
        this.cone_unknow = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Map
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cone_yellow]
    // Serialize the length for message field [cone_yellow]
    bufferOffset = _serializer.uint32(obj.cone_yellow.length, buffer, bufferOffset);
    obj.cone_yellow.forEach((val) => {
      bufferOffset = Cone.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cone_blue]
    // Serialize the length for message field [cone_blue]
    bufferOffset = _serializer.uint32(obj.cone_blue.length, buffer, bufferOffset);
    obj.cone_blue.forEach((val) => {
      bufferOffset = Cone.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [orange]
    // Serialize the length for message field [orange]
    bufferOffset = _serializer.uint32(obj.orange.length, buffer, bufferOffset);
    obj.orange.forEach((val) => {
      bufferOffset = Cone.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [cone_unknow]
    // Serialize the length for message field [cone_unknow]
    bufferOffset = _serializer.uint32(obj.cone_unknow.length, buffer, bufferOffset);
    obj.cone_unknow.forEach((val) => {
      bufferOffset = Cone.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Map
    let len;
    let data = new Map(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cone_yellow]
    // Deserialize array length for message field [cone_yellow]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cone_yellow = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cone_yellow[i] = Cone.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cone_blue]
    // Deserialize array length for message field [cone_blue]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cone_blue = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cone_blue[i] = Cone.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [orange]
    // Deserialize array length for message field [orange]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.orange = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.orange[i] = Cone.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [cone_unknow]
    // Deserialize array length for message field [cone_unknow]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cone_unknow = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cone_unknow[i] = Cone.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.cone_yellow.forEach((val) => {
      length += Cone.getMessageSize(val);
    });
    object.cone_blue.forEach((val) => {
      length += Cone.getMessageSize(val);
    });
    object.orange.forEach((val) => {
      length += Cone.getMessageSize(val);
    });
    object.cone_unknow.forEach((val) => {
      length += Cone.getMessageSize(val);
    });
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'aam_common_msgs/Map';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd926454e54ff8605a5b5a6710745bd29';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    std_msgs/Header header
    
    aam_common_msgs/Cone[] cone_yellow
    aam_common_msgs/Cone[] cone_blue
    aam_common_msgs/Cone[] orange
    aam_common_msgs/Cone[] cone_unknow
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: aam_common_msgs/Cone
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
    const resolved = new Map(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cone_yellow !== undefined) {
      resolved.cone_yellow = new Array(msg.cone_yellow.length);
      for (let i = 0; i < resolved.cone_yellow.length; ++i) {
        resolved.cone_yellow[i] = Cone.Resolve(msg.cone_yellow[i]);
      }
    }
    else {
      resolved.cone_yellow = []
    }

    if (msg.cone_blue !== undefined) {
      resolved.cone_blue = new Array(msg.cone_blue.length);
      for (let i = 0; i < resolved.cone_blue.length; ++i) {
        resolved.cone_blue[i] = Cone.Resolve(msg.cone_blue[i]);
      }
    }
    else {
      resolved.cone_blue = []
    }

    if (msg.orange !== undefined) {
      resolved.orange = new Array(msg.orange.length);
      for (let i = 0; i < resolved.orange.length; ++i) {
        resolved.orange[i] = Cone.Resolve(msg.orange[i]);
      }
    }
    else {
      resolved.orange = []
    }

    if (msg.cone_unknow !== undefined) {
      resolved.cone_unknow = new Array(msg.cone_unknow.length);
      for (let i = 0; i < resolved.cone_unknow.length; ++i) {
        resolved.cone_unknow[i] = Cone.Resolve(msg.cone_unknow[i]);
      }
    }
    else {
      resolved.cone_unknow = []
    }

    return resolved;
    }
};

module.exports = Map;
