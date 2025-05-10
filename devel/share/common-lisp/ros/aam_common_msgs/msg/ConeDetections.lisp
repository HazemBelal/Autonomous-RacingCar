; Auto-generated. Do not edit!


(cl:in-package aam_common_msgs-msg)


;//! \htmlinclude ConeDetections.msg.html

(cl:defclass <ConeDetections> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cone_detections
    :reader cone_detections
    :initarg :cone_detections
    :type (cl:vector aam_common_msgs-msg:Cone)
   :initform (cl:make-array 0 :element-type 'aam_common_msgs-msg:Cone :initial-element (cl:make-instance 'aam_common_msgs-msg:Cone))))
)

(cl:defclass ConeDetections (<ConeDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConeDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConeDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aam_common_msgs-msg:<ConeDetections> is deprecated: use aam_common_msgs-msg:ConeDetections instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConeDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aam_common_msgs-msg:header-val is deprecated.  Use aam_common_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cone_detections-val :lambda-list '(m))
(cl:defmethod cone_detections-val ((m <ConeDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aam_common_msgs-msg:cone_detections-val is deprecated.  Use aam_common_msgs-msg:cone_detections instead.")
  (cone_detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConeDetections>) ostream)
  "Serializes a message object of type '<ConeDetections>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cone_detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cone_detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConeDetections>) istream)
  "Deserializes a message object of type '<ConeDetections>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cone_detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cone_detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'aam_common_msgs-msg:Cone))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConeDetections>)))
  "Returns string type for a message object of type '<ConeDetections>"
  "aam_common_msgs/ConeDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConeDetections)))
  "Returns string type for a message object of type 'ConeDetections"
  "aam_common_msgs/ConeDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConeDetections>)))
  "Returns md5sum for a message object of type '<ConeDetections>"
  "98c8c85adf89b9e9d5d8cd2b615e2a7e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConeDetections)))
  "Returns md5sum for a message object of type 'ConeDetections"
  "98c8c85adf89b9e9d5d8cd2b615e2a7e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConeDetections>)))
  "Returns full string definition for message of type '<ConeDetections>"
  (cl:format cl:nil "std_msgs/Header header~%~%aam_common_msgs/Cone[] cone_detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: aam_common_msgs/Cone~%geometry_msgs/Point position   # coordinate of cone in [x, y]~%std_msgs/String color   # color of cone, 'b' = blue, 'y' = yellow, 'so' = orange , 'bo' = orange~%float64 ratioX~%float64 ratioY~%float64 pcX~%float64 pcY~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConeDetections)))
  "Returns full string definition for message of type 'ConeDetections"
  (cl:format cl:nil "std_msgs/Header header~%~%aam_common_msgs/Cone[] cone_detections~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: aam_common_msgs/Cone~%geometry_msgs/Point position   # coordinate of cone in [x, y]~%std_msgs/String color   # color of cone, 'b' = blue, 'y' = yellow, 'so' = orange , 'bo' = orange~%float64 ratioX~%float64 ratioY~%float64 pcX~%float64 pcY~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: std_msgs/String~%string data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConeDetections>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cone_detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConeDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'ConeDetections
    (cl:cons ':header (header msg))
    (cl:cons ':cone_detections (cone_detections msg))
))
