;; Auto-generated. Do not edit!


(when (boundp 'navigation2::auto)
  (if (not (find-package "NAVIGATION2"))
    (make-package "NAVIGATION2"))
  (shadow 'auto (find-package "NAVIGATION2")))
(unless (find-package "NAVIGATION2::AUTO")
  (make-package "NAVIGATION2::AUTO"))

(in-package "ROS")
;;//! \htmlinclude auto.msg.html


(defclass navigation2::auto
  :super ros::object
  :slots (_arm _latitude _longitude _setstage _text _aruco_coordinates _reference _msg_id ))

(defmethod navigation2::auto
  (:init
   (&key
    ((:arm __arm) nil)
    ((:latitude __latitude) 0.0)
    ((:longitude __longitude) 0.0)
    ((:setstage __setstage) 0)
    ((:text __text) "")
    ((:aruco_coordinates __aruco_coordinates) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:reference __reference) "")
    ((:msg_id __msg_id) 0)
    )
   (send-super :init)
   (setq _arm __arm)
   (setq _latitude (float __latitude))
   (setq _longitude (float __longitude))
   (setq _setstage (round __setstage))
   (setq _text (string __text))
   (setq _aruco_coordinates __aruco_coordinates)
   (setq _reference (string __reference))
   (setq _msg_id (round __msg_id))
   self)
  (:arm
   (&optional (__arm :null))
   (if (not (eq __arm :null)) (setq _arm __arm)) _arm)
  (:latitude
   (&optional __latitude)
   (if __latitude (setq _latitude __latitude)) _latitude)
  (:longitude
   (&optional __longitude)
   (if __longitude (setq _longitude __longitude)) _longitude)
  (:setstage
   (&optional __setstage)
   (if __setstage (setq _setstage __setstage)) _setstage)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:aruco_coordinates
   (&optional __aruco_coordinates)
   (if __aruco_coordinates (setq _aruco_coordinates __aruco_coordinates)) _aruco_coordinates)
  (:reference
   (&optional __reference)
   (if __reference (setq _reference __reference)) _reference)
  (:msg_id
   (&optional __msg_id)
   (if __msg_id (setq _msg_id __msg_id)) _msg_id)
  (:serialization-length
   ()
   (+
    ;; bool _arm
    1
    ;; float64 _latitude
    8
    ;; float64 _longitude
    8
    ;; int8 _setstage
    1
    ;; string _text
    4 (length _text)
    ;; float64[] _aruco_coordinates
    (* 8    (length _aruco_coordinates)) 4
    ;; string _reference
    4 (length _reference)
    ;; int8 _msg_id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _arm
       (if _arm (write-byte -1 s) (write-byte 0 s))
     ;; float64 _latitude
       (sys::poke _latitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _longitude
       (sys::poke _longitude (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _setstage
       (write-byte _setstage s)
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;; float64[] _aruco_coordinates
     (write-long (length _aruco_coordinates) s)
     (dotimes (i (length _aruco_coordinates))
       (sys::poke (elt _aruco_coordinates i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; string _reference
       (write-long (length _reference) s) (princ _reference s)
     ;; int8 _msg_id
       (write-byte _msg_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _arm
     (setq _arm (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _latitude
     (setq _latitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _longitude
     (setq _longitude (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _setstage
     (setq _setstage (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _setstage 127) (setq _setstage (- _setstage 256)))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64[] _aruco_coordinates
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _aruco_coordinates (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _aruco_coordinates i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; string _reference
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _reference (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int8 _msg_id
     (setq _msg_id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _msg_id 127) (setq _msg_id (- _msg_id 256)))
   ;;
   self)
  )

(setf (get navigation2::auto :md5sum-) "65672b1bd9673047ebc0672e3417ec0f")
(setf (get navigation2::auto :datatype-) "navigation2/auto")
(setf (get navigation2::auto :definition-)
      "bool arm
float64 latitude 
float64 longitude
int8 setstage 
string text
float64[] aruco_coordinates
string reference 
int8 msg_id


")



(provide :navigation2/auto "65672b1bd9673047ebc0672e3417ec0f")


