;; Auto-generated. Do not edit!


(when (boundp 'navigation2::Goal)
  (if (not (find-package "NAVIGATION2"))
    (make-package "NAVIGATION2"))
  (shadow 'Goal (find-package "NAVIGATION2")))
(unless (find-package "NAVIGATION2::GOAL")
  (make-package "NAVIGATION2::GOAL"))

(in-package "ROS")
;;//! \htmlinclude Goal.msg.html


(defclass navigation2::Goal
  :super ros::object
  :slots (_bearing _distance ))

(defmethod navigation2::Goal
  (:init
   (&key
    ((:bearing __bearing) 0.0)
    ((:distance __distance) 0.0)
    )
   (send-super :init)
   (setq _bearing (float __bearing))
   (setq _distance (float __distance))
   self)
  (:bearing
   (&optional __bearing)
   (if __bearing (setq _bearing __bearing)) _bearing)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:serialization-length
   ()
   (+
    ;; float64 _bearing
    8
    ;; float64 _distance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _bearing
       (sys::poke _bearing (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _bearing
     (setq _bearing (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distance
     (setq _distance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get navigation2::Goal :md5sum-) "e6b9899956f1eacd3991a6d711941249")
(setf (get navigation2::Goal :datatype-) "navigation2/Goal")
(setf (get navigation2::Goal :definition-)
      "float64 bearing
float64 distance

")



(provide :navigation2/Goal "e6b9899956f1eacd3991a6d711941249")


