;; Auto-generated. Do not edit!


(when (boundp 'navigation2::red)
  (if (not (find-package "NAVIGATION2"))
    (make-package "NAVIGATION2"))
  (shadow 'red (find-package "NAVIGATION2")))
(unless (find-package "NAVIGATION2::RED")
  (make-package "NAVIGATION2::RED"))

(in-package "ROS")
;;//! \htmlinclude red.msg.html


(defclass navigation2::red
  :super ros::object
  :slots (_vel _omega _detect ))

(defmethod navigation2::red
  (:init
   (&key
    ((:vel __vel) 0)
    ((:omega __omega) 0)
    ((:detect __detect) nil)
    )
   (send-super :init)
   (setq _vel (round __vel))
   (setq _omega (round __omega))
   (setq _detect __detect)
   self)
  (:vel
   (&optional __vel)
   (if __vel (setq _vel __vel)) _vel)
  (:omega
   (&optional __omega)
   (if __omega (setq _omega __omega)) _omega)
  (:detect
   (&optional (__detect :null))
   (if (not (eq __detect :null)) (setq _detect __detect)) _detect)
  (:serialization-length
   ()
   (+
    ;; int16 _vel
    2
    ;; int16 _omega
    2
    ;; bool _detect
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int16 _vel
       (write-word _vel s)
     ;; int16 _omega
       (write-word _omega s)
     ;; bool _detect
       (if _detect (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int16 _vel
     (setq _vel (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; int16 _omega
     (setq _omega (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _detect
     (setq _detect (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get navigation2::red :md5sum-) "598737a047b4631c88d712e78758247d")
(setf (get navigation2::red :datatype-) "navigation2/red")
(setf (get navigation2::red :definition-)
      "int16 vel
int16 omega
bool detect


")



(provide :navigation2/red "598737a047b4631c88d712e78758247d")


