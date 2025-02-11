;; Auto-generated. Do not edit!


(when (boundp 'navigation2::detection)
  (if (not (find-package "NAVIGATION2"))
    (make-package "NAVIGATION2"))
  (shadow 'detection (find-package "NAVIGATION2")))
(unless (find-package "NAVIGATION2::DETECTION")
  (make-package "NAVIGATION2::DETECTION"))

(in-package "ROS")
;;//! \htmlinclude detection.msg.html


(defclass navigation2::detection
  :super ros::object
  :slots (_color _depth ))

(defmethod navigation2::detection
  (:init
   (&key
    ((:color __color) "")
    ((:depth __depth) 0.0)
    )
   (send-super :init)
   (setq _color (string __color))
   (setq _depth (float __depth))
   self)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:depth
   (&optional __depth)
   (if __depth (setq _depth __depth)) _depth)
  (:serialization-length
   ()
   (+
    ;; string _color
    4 (length _color)
    ;; float32 _depth
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;; float32 _depth
       (sys::poke _depth (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float32 _depth
     (setq _depth (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get navigation2::detection :md5sum-) "54cdd292028a384f7f8301d873bb5638")
(setf (get navigation2::detection :datatype-) "navigation2/detection")
(setf (get navigation2::detection :definition-)
      "string color
float32 depth

")



(provide :navigation2/detection "54cdd292028a384f7f8301d873bb5638")


