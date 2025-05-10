;; Auto-generated. Do not edit!


(when (boundp 'aam_common_msgs::Cone)
  (if (not (find-package "AAM_COMMON_MSGS"))
    (make-package "AAM_COMMON_MSGS"))
  (shadow 'Cone (find-package "AAM_COMMON_MSGS")))
(unless (find-package "AAM_COMMON_MSGS::CONE")
  (make-package "AAM_COMMON_MSGS::CONE"))

(in-package "ROS")
;;//! \htmlinclude Cone.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass aam_common_msgs::Cone
  :super ros::object
  :slots (_position _color _ratioX _ratioY _pcX _pcY ))

(defmethod aam_common_msgs::Cone
  (:init
   (&key
    ((:position __position) (instance geometry_msgs::Point :init))
    ((:color __color) (instance std_msgs::String :init))
    ((:ratioX __ratioX) 0.0)
    ((:ratioY __ratioY) 0.0)
    ((:pcX __pcX) 0.0)
    ((:pcY __pcY) 0.0)
    )
   (send-super :init)
   (setq _position __position)
   (setq _color __color)
   (setq _ratioX (float __ratioX))
   (setq _ratioY (float __ratioY))
   (setq _pcX (float __pcX))
   (setq _pcY (float __pcY))
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:ratioX
   (&optional __ratioX)
   (if __ratioX (setq _ratioX __ratioX)) _ratioX)
  (:ratioY
   (&optional __ratioY)
   (if __ratioY (setq _ratioY __ratioY)) _ratioY)
  (:pcX
   (&optional __pcX)
   (if __pcX (setq _pcX __pcX)) _pcX)
  (:pcY
   (&optional __pcY)
   (if __pcY (setq _pcY __pcY)) _pcY)
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point _position
    (send _position :serialization-length)
    ;; std_msgs/String _color
    (send _color :serialization-length)
    ;; float64 _ratioX
    8
    ;; float64 _ratioY
    8
    ;; float64 _pcX
    8
    ;; float64 _pcY
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point _position
       (send _position :serialize s)
     ;; std_msgs/String _color
       (send _color :serialize s)
     ;; float64 _ratioX
       (sys::poke _ratioX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _ratioY
       (sys::poke _ratioY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pcX
       (sys::poke _pcX (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _pcY
       (sys::poke _pcY (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; std_msgs/String _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;; float64 _ratioX
     (setq _ratioX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _ratioY
     (setq _ratioY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pcX
     (setq _pcX (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _pcY
     (setq _pcY (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get aam_common_msgs::Cone :md5sum-) "0f501329c9e6188fb393747c57c79cb9")
(setf (get aam_common_msgs::Cone :datatype-) "aam_common_msgs/Cone")
(setf (get aam_common_msgs::Cone :definition-)
      "geometry_msgs/Point position   # coordinate of cone in [x, y]
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



(provide :aam_common_msgs/Cone "0f501329c9e6188fb393747c57c79cb9")


