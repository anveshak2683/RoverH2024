;; Auto-generated. Do not edit!


(when (boundp 'navigation2::Enc_dist)
  (if (not (find-package "NAVIGATION2"))
    (make-package "NAVIGATION2"))
  (shadow 'Enc_dist (find-package "NAVIGATION2")))
(unless (find-package "NAVIGATION2::ENC_DIST")
  (make-package "NAVIGATION2::ENC_DIST"))

(in-package "ROS")
;;//! \htmlinclude Enc_dist.msg.html


(defclass navigation2::Enc_dist
  :super ros::object
  :slots (_dist ))

(defmethod navigation2::Enc_dist
  (:init
   (&key
    ((:dist __dist) 0.0)
    )
   (send-super :init)
   (setq _dist (float __dist))
   self)
  (:dist
   (&optional __dist)
   (if __dist (setq _dist __dist)) _dist)
  (:serialization-length
   ()
   (+
    ;; float64 _dist
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _dist
       (sys::poke _dist (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _dist
     (setq _dist (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get navigation2::Enc_dist :md5sum-) "3f4fece6412db25720b2bab9f80b3473")
(setf (get navigation2::Enc_dist :datatype-) "navigation2/Enc_dist")
(setf (get navigation2::Enc_dist :definition-)
      "float64 dist

")



(provide :navigation2/Enc_dist "3f4fece6412db25720b2bab9f80b3473")


