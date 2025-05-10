; Auto-generated. Do not edit!


(cl:in-package ma_rrt_path_plan-msg)


;//! \htmlinclude WaypointsArray.msg.html

(cl:defclass <WaypointsArray> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (preliminaryLoopClosure
    :reader preliminaryLoopClosure
    :initarg :preliminaryLoopClosure
    :type cl:boolean
    :initform cl:nil)
   (loopClosure
    :reader loopClosure
    :initarg :loopClosure
    :type cl:boolean
    :initform cl:nil)
   (waypoints
    :reader waypoints
    :initarg :waypoints
    :type (cl:vector ma_rrt_path_plan-msg:Waypoint)
   :initform (cl:make-array 0 :element-type 'ma_rrt_path_plan-msg:Waypoint :initial-element (cl:make-instance 'ma_rrt_path_plan-msg:Waypoint))))
)

(cl:defclass WaypointsArray (<WaypointsArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WaypointsArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WaypointsArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ma_rrt_path_plan-msg:<WaypointsArray> is deprecated: use ma_rrt_path_plan-msg:WaypointsArray instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WaypointsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ma_rrt_path_plan-msg:header-val is deprecated.  Use ma_rrt_path_plan-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'preliminaryLoopClosure-val :lambda-list '(m))
(cl:defmethod preliminaryLoopClosure-val ((m <WaypointsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ma_rrt_path_plan-msg:preliminaryLoopClosure-val is deprecated.  Use ma_rrt_path_plan-msg:preliminaryLoopClosure instead.")
  (preliminaryLoopClosure m))

(cl:ensure-generic-function 'loopClosure-val :lambda-list '(m))
(cl:defmethod loopClosure-val ((m <WaypointsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ma_rrt_path_plan-msg:loopClosure-val is deprecated.  Use ma_rrt_path_plan-msg:loopClosure instead.")
  (loopClosure m))

(cl:ensure-generic-function 'waypoints-val :lambda-list '(m))
(cl:defmethod waypoints-val ((m <WaypointsArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ma_rrt_path_plan-msg:waypoints-val is deprecated.  Use ma_rrt_path_plan-msg:waypoints instead.")
  (waypoints m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WaypointsArray>) ostream)
  "Serializes a message object of type '<WaypointsArray>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'preliminaryLoopClosure) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'loopClosure) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'waypoints))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'waypoints))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WaypointsArray>) istream)
  "Deserializes a message object of type '<WaypointsArray>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'preliminaryLoopClosure) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'loopClosure) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'waypoints) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'waypoints)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'ma_rrt_path_plan-msg:Waypoint))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WaypointsArray>)))
  "Returns string type for a message object of type '<WaypointsArray>"
  "ma_rrt_path_plan/WaypointsArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WaypointsArray)))
  "Returns string type for a message object of type 'WaypointsArray"
  "ma_rrt_path_plan/WaypointsArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WaypointsArray>)))
  "Returns md5sum for a message object of type '<WaypointsArray>"
  "f11df5cc21094a3f66524c757aa5cb93")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WaypointsArray)))
  "Returns md5sum for a message object of type 'WaypointsArray"
  "f11df5cc21094a3f66524c757aa5cb93")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WaypointsArray>)))
  "Returns full string definition for message of type '<WaypointsArray>"
  (cl:format cl:nil "std_msgs/Header header~%bool preliminaryLoopClosure~%bool loopClosure~%Waypoint[] waypoints~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: ma_rrt_path_plan/Waypoint~%int32 id~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WaypointsArray)))
  "Returns full string definition for message of type 'WaypointsArray"
  (cl:format cl:nil "std_msgs/Header header~%bool preliminaryLoopClosure~%bool loopClosure~%Waypoint[] waypoints~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: ma_rrt_path_plan/Waypoint~%int32 id~%float64 x~%float64 y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WaypointsArray>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'waypoints) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WaypointsArray>))
  "Converts a ROS message object to a list"
  (cl:list 'WaypointsArray
    (cl:cons ':header (header msg))
    (cl:cons ':preliminaryLoopClosure (preliminaryLoopClosure msg))
    (cl:cons ':loopClosure (loopClosure msg))
    (cl:cons ':waypoints (waypoints msg))
))
