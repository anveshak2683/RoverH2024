;; Auto-generated. Do not edit!


(when (boundp 'navigation2::enc_feed)
  (if (not (find-package "NAVIGATION2"))
    (make-package "NAVIGATION2"))
  (shadow 'enc_feed (find-package "NAVIGATION2")))
(unless (find-package "NAVIGATION2::ENC_FEED")
  (make-package "NAVIGATION2::ENC_FEED"))

(in-package "ROS")
;;//! \htmlinclude enc_feed.msg.html


(defclass navigation2::enc_feed
  :super ros::object
  :slots (_vel _angle ))

(defmethod navigation2::enc_feed
  (:init
   (&key
    ((:vel __vel) 0)
    ((:angle __angle) 0)
    )
   (send-super :init)
   (setq _vel (round __vel))
   (setq _angle (round __angle))
   self)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; int16 _vel
    2
    ;; int16 _angle
    2
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _vel
       (write-word _vel s)
     ;; int16 _angle
       (write-word _angle s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _vel
     (setq _vel (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _angle
     (setq _angle (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;;
   self)
  )

(setf (get navigation2::enc_feed :md5sum-) "c7d324c5346393c6bc385a7a281fa92c")
(setf (get navigation2::enc_feed :datatype-) "navigation2/enc_feed")
(setf (get navigation2::enc_feed :definition-)
      "int16 vel
int16 angle
")



(provide :navigation2/enc_feed "c7d324c5346393c6bc385a7a281fa92c")


