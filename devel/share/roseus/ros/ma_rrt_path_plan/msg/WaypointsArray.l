;; Auto-generated. Do not edit!


(when (boundp 'ma_rrt_path_plan::WaypointsArray)
  (if (not (find-package "MA_RRT_PATH_PLAN"))
    (make-package "MA_RRT_PATH_PLAN"))
  (shadow 'WaypointsArray (find-package "MA_RRT_PATH_PLAN")))
(unless (find-package "MA_RRT_PATH_PLAN::WAYPOINTSARRAY")
  (make-package "MA_RRT_PATH_PLAN::WAYPOINTSARRAY"))

(in-package "ROS")
;;//! \htmlinclude WaypointsArray.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass ma_rrt_path_plan::WaypointsArray
  :super ros::object
  :slots (_header _preliminaryLoopClosure _loopClosure _waypoints ))

(defmethod ma_rrt_path_plan::WaypointsArray
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:preliminaryLoopClosure __preliminaryLoopClosure) nil)
    ((:loopClosure __loopClosure) nil)
    ((:waypoints __waypoints) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _preliminaryLoopClosure __preliminaryLoopClosure)
   (setq _loopClosure __loopClosure)
   (setq _waypoints __waypoints)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:preliminaryLoopClosure
   (&optional (__preliminaryLoopClosure :null))
   (if (not (eq __preliminaryLoopClosure :null)) (setq _preliminaryLoopClosure __preliminaryLoopClosure)) _preliminaryLoopClosure)
  (:loopClosure
   (&optional (__loopClosure :null))
   (if (not (eq __loopClosure :null)) (setq _loopClosure __loopClosure)) _loopClosure)
  (:waypoints
   (&rest __waypoints)
   (if (keywordp (car __waypoints))
       (send* _waypoints __waypoints)
     (progn
       (if __waypoints (setq _waypoints (car __waypoints)))
       _waypoints)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bool _preliminaryLoopClosure
    1
    ;; bool _loopClosure
    1
    ;; ma_rrt_path_plan/Waypoint[] _waypoints
    (apply #'+ (send-all _waypoints :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bool _preliminaryLoopClosure
       (if _preliminaryLoopClosure (write-byte -1 s) (write-byte 0 s))
     ;; bool _loopClosure
       (if _loopClosure (write-byte -1 s) (write-byte 0 s))
     ;; ma_rrt_path_plan/Waypoint[] _waypoints
     (write-long (length _waypoints) s)
     (dolist (elem _waypoints)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bool _preliminaryLoopClosure
     (setq _preliminaryLoopClosure (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; bool _loopClosure
     (setq _loopClosure (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; ma_rrt_path_plan/Waypoint[] _waypoints
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _waypoints (let (r) (dotimes (i n) (push (instance ma_rrt_path_plan::Waypoint :init) r)) r))
     (dolist (elem- _waypoints)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get ma_rrt_path_plan::WaypointsArray :md5sum-) "f11df5cc21094a3f66524c757aa5cb93")
(setf (get ma_rrt_path_plan::WaypointsArray :datatype-) "ma_rrt_path_plan/WaypointsArray")
(setf (get ma_rrt_path_plan::WaypointsArray :definition-)
      "std_msgs/Header header
bool preliminaryLoopClosure
bool loopClosure
Waypoint[] waypoints
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
MSG: ma_rrt_path_plan/Waypoint
int32 id
float64 x
float64 y

")



(provide :ma_rrt_path_plan/WaypointsArray "f11df5cc21094a3f66524c757aa5cb93")


