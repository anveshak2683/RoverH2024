;; Auto-generated. Do not edit!


(when (boundp 'navigation2::imu_angle)
  (if (not (find-package "NAVIGATION2"))
    (make-package "NAVIGATION2"))
  (shadow 'imu_angle (find-package "NAVIGATION2")))
(unless (find-package "NAVIGATION2::IMU_ANGLE")
  (make-package "NAVIGATION2::IMU_ANGLE"))

(in-package "ROS")
;;//! \htmlinclude imu_angle.msg.html


(defclass navigation2::imu_angle
  :super ros::object
  :slots (_Roll _Pitch _Yaw ))

(defmethod navigation2::imu_angle
  (:init
   (&key
    ((:Roll __Roll) 0.0)
    ((:Pitch __Pitch) 0.0)
    ((:Yaw __Yaw) 0.0)
    )
   (send-super :init)
   (setq _Roll (float __Roll))
   (setq _Pitch (float __Pitch))
   (setq _Yaw (float __Yaw))
   self)
  (:Roll
   (&optional __Roll)
   (if __Roll (setq _Roll __Roll)) _Roll)
  (:Pitch
   (&optional __Pitch)
   (if __Pitch (setq _Pitch __Pitch)) _Pitch)
  (:Yaw
   (&optional __Yaw)
   (if __Yaw (setq _Yaw __Yaw)) _Yaw)
  (:serialization-length
   ()
   (+
    ;; float32 _Roll
    4
    ;; float32 _Pitch
    4
    ;; float32 _Yaw
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _Roll
       (sys::poke _Roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Pitch
       (sys::poke _Pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _Yaw
       (sys::poke _Yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _Roll
     (setq _Roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Pitch
     (setq _Pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _Yaw
     (setq _Yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get navigation2::imu_angle :md5sum-) "d8aec502c207f580f351d8f6036c0af0")
(setf (get navigation2::imu_angle :datatype-) "navigation2/imu_angle")
(setf (get navigation2::imu_angle :definition-)
      "float32 Roll
float32 Pitch
float32 Yaw

")



(provide :navigation2/imu_angle "d8aec502c207f580f351d8f6036c0af0")


