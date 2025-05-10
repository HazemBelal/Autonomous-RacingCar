;; Auto-generated. Do not edit!


(when (boundp 'ma_rrt_path_plan::Waypoint)
  (if (not (find-package "MA_RRT_PATH_PLAN"))
    (make-package "MA_RRT_PATH_PLAN"))
  (shadow 'Waypoint (find-package "MA_RRT_PATH_PLAN")))
(unless (find-package "MA_RRT_PATH_PLAN::WAYPOINT")
  (make-package "MA_RRT_PATH_PLAN::WAYPOINT"))

(in-package "ROS")
;;//! \htmlinclude Waypoint.msg.html


(defclass ma_rrt_path_plan::Waypoint
  :super ros::object
  :slots (_id _x _y ))

(defmethod ma_rrt_path_plan::Waypoint
  (:init
   (&key
    ((:id __id) 0)
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _x (float __x))
   (setq _y (float __y))
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; float64 _x
    8
    ;; float64 _y
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; float64 _x
       (sys::poke _x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _y
       (sys::poke _y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _x
     (setq _x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _y
     (setq _y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get ma_rrt_path_plan::Waypoint :md5sum-) "0df65d6dff578c1933cd201b742cef45")
(setf (get ma_rrt_path_plan::Waypoint :datatype-) "ma_rrt_path_plan/Waypoint")
(setf (get ma_rrt_path_plan::Waypoint :definition-)
      "int32 id
float64 x
float64 y

")



(provide :ma_rrt_path_plan/Waypoint "0df65d6dff578c1933cd201b742cef45")


