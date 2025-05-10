;; Auto-generated. Do not edit!


(when (boundp 'aam_common_msgs::Map)
  (if (not (find-package "AAM_COMMON_MSGS"))
    (make-package "AAM_COMMON_MSGS"))
  (shadow 'Map (find-package "AAM_COMMON_MSGS")))
(unless (find-package "AAM_COMMON_MSGS::MAP")
  (make-package "AAM_COMMON_MSGS::MAP"))

(in-package "ROS")
;;//! \htmlinclude Map.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass aam_common_msgs::Map
  :super ros::object
  :slots (_header _cone_yellow _cone_blue _orange _cone_unknow ))

(defmethod aam_common_msgs::Map
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cone_yellow __cone_yellow) ())
    ((:cone_blue __cone_blue) ())
    ((:orange __orange) ())
    ((:cone_unknow __cone_unknow) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _cone_yellow __cone_yellow)
   (setq _cone_blue __cone_blue)
   (setq _orange __orange)
   (setq _cone_unknow __cone_unknow)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cone_yellow
   (&rest __cone_yellow)
   (if (keywordp (car __cone_yellow))
       (send* _cone_yellow __cone_yellow)
     (progn
       (if __cone_yellow (setq _cone_yellow (car __cone_yellow)))
       _cone_yellow)))
  (:cone_blue
   (&rest __cone_blue)
   (if (keywordp (car __cone_blue))
       (send* _cone_blue __cone_blue)
     (progn
       (if __cone_blue (setq _cone_blue (car __cone_blue)))
       _cone_blue)))
  (:orange
   (&rest __orange)
   (if (keywordp (car __orange))
       (send* _orange __orange)
     (progn
       (if __orange (setq _orange (car __orange)))
       _orange)))
  (:cone_unknow
   (&rest __cone_unknow)
   (if (keywordp (car __cone_unknow))
       (send* _cone_unknow __cone_unknow)
     (progn
       (if __cone_unknow (setq _cone_unknow (car __cone_unknow)))
       _cone_unknow)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; aam_common_msgs/Cone[] _cone_yellow
    (apply #'+ (send-all _cone_yellow :serialization-length)) 4
    ;; aam_common_msgs/Cone[] _cone_blue
    (apply #'+ (send-all _cone_blue :serialization-length)) 4
    ;; aam_common_msgs/Cone[] _orange
    (apply #'+ (send-all _orange :serialization-length)) 4
    ;; aam_common_msgs/Cone[] _cone_unknow
    (apply #'+ (send-all _cone_unknow :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; aam_common_msgs/Cone[] _cone_yellow
     (write-long (length _cone_yellow) s)
     (dolist (elem _cone_yellow)
       (send elem :serialize s)
       )
     ;; aam_common_msgs/Cone[] _cone_blue
     (write-long (length _cone_blue) s)
     (dolist (elem _cone_blue)
       (send elem :serialize s)
       )
     ;; aam_common_msgs/Cone[] _orange
     (write-long (length _orange) s)
     (dolist (elem _orange)
       (send elem :serialize s)
       )
     ;; aam_common_msgs/Cone[] _cone_unknow
     (write-long (length _cone_unknow) s)
     (dolist (elem _cone_unknow)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; aam_common_msgs/Cone[] _cone_yellow
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cone_yellow (let (r) (dotimes (i n) (push (instance aam_common_msgs::Cone :init) r)) r))
     (dolist (elem- _cone_yellow)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; aam_common_msgs/Cone[] _cone_blue
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cone_blue (let (r) (dotimes (i n) (push (instance aam_common_msgs::Cone :init) r)) r))
     (dolist (elem- _cone_blue)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; aam_common_msgs/Cone[] _orange
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _orange (let (r) (dotimes (i n) (push (instance aam_common_msgs::Cone :init) r)) r))
     (dolist (elem- _orange)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; aam_common_msgs/Cone[] _cone_unknow
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cone_unknow (let (r) (dotimes (i n) (push (instance aam_common_msgs::Cone :init) r)) r))
     (dolist (elem- _cone_unknow)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get aam_common_msgs::Map :md5sum-) "d926454e54ff8605a5b5a6710745bd29")
(setf (get aam_common_msgs::Map :datatype-) "aam_common_msgs/Map")
(setf (get aam_common_msgs::Map :definition-)
      "std_msgs/Header header

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

")



(provide :aam_common_msgs/Map "d926454e54ff8605a5b5a6710745bd29")


