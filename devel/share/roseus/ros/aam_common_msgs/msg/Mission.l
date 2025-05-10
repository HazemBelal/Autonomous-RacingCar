;; Auto-generated. Do not edit!


(when (boundp 'aam_common_msgs::Mission)
  (if (not (find-package "AAM_COMMON_MSGS"))
    (make-package "AAM_COMMON_MSGS"))
  (shadow 'Mission (find-package "AAM_COMMON_MSGS")))
(unless (find-package "AAM_COMMON_MSGS::MISSION")
  (make-package "AAM_COMMON_MSGS::MISSION"))

(in-package "ROS")
;;//! \htmlinclude Mission.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass aam_common_msgs::Mission
  :super ros::object
  :slots (_header _mission _finished ))

(defmethod aam_common_msgs::Mission
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:mission __mission) "")
    ((:finished __finished) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _mission (string __mission))
   (setq _finished __finished)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:mission
   (&optional __mission)
   (if __mission (setq _mission __mission)) _mission)
  (:finished
   (&optional (__finished :null))
   (if (not (eq __finished :null)) (setq _finished __finished)) _finished)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _mission
    4 (length _mission)
    ;; bool _finished
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _mission
       (write-long (length _mission) s) (princ _mission s)
     ;; bool _finished
       (if _finished (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _mission
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mission (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _finished
     (setq _finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get aam_common_msgs::Mission :md5sum-) "aa152a842fbf6b840a20d415a574e050")
(setf (get aam_common_msgs::Mission :datatype-) "aam_common_msgs/Mission")
(setf (get aam_common_msgs::Mission :definition-)
      "# Header
std_msgs/Header header
string mission
bool finished
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

")



(provide :aam_common_msgs/Mission "aa152a842fbf6b840a20d415a574e050")


