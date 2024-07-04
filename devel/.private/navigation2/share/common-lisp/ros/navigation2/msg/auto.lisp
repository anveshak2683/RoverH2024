; Auto-generated. Do not edit!


(cl:in-package navigation2-msg)


;//! \htmlinclude auto.msg.html

(cl:defclass <auto> (roslisp-msg-protocol:ros-message)
  ((arm
    :reader arm
    :initarg :arm
    :type cl:boolean
    :initform cl:nil)
   (latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0)
   (setstage
    :reader setstage
    :initarg :setstage
    :type cl:fixnum
    :initform 0)
   (text
    :reader text
    :initarg :text
    :type cl:string
    :initform "")
   (aruco_coordinates
    :reader aruco_coordinates
    :initarg :aruco_coordinates
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (reference
    :reader reference
    :initarg :reference
    :type cl:string
    :initform "")
   (msg_id
    :reader msg_id
    :initarg :msg_id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass auto (<auto>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <auto>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'auto)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation2-msg:<auto> is deprecated: use navigation2-msg:auto instead.")))

(cl:ensure-generic-function 'arm-val :lambda-list '(m))
(cl:defmethod arm-val ((m <auto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:arm-val is deprecated.  Use navigation2-msg:arm instead.")
  (arm m))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <auto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:latitude-val is deprecated.  Use navigation2-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <auto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:longitude-val is deprecated.  Use navigation2-msg:longitude instead.")
  (longitude m))

(cl:ensure-generic-function 'setstage-val :lambda-list '(m))
(cl:defmethod setstage-val ((m <auto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:setstage-val is deprecated.  Use navigation2-msg:setstage instead.")
  (setstage m))

(cl:ensure-generic-function 'text-val :lambda-list '(m))
(cl:defmethod text-val ((m <auto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:text-val is deprecated.  Use navigation2-msg:text instead.")
  (text m))

(cl:ensure-generic-function 'aruco_coordinates-val :lambda-list '(m))
(cl:defmethod aruco_coordinates-val ((m <auto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:aruco_coordinates-val is deprecated.  Use navigation2-msg:aruco_coordinates instead.")
  (aruco_coordinates m))

(cl:ensure-generic-function 'reference-val :lambda-list '(m))
(cl:defmethod reference-val ((m <auto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:reference-val is deprecated.  Use navigation2-msg:reference instead.")
  (reference m))

(cl:ensure-generic-function 'msg_id-val :lambda-list '(m))
(cl:defmethod msg_id-val ((m <auto>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:msg_id-val is deprecated.  Use navigation2-msg:msg_id instead.")
  (msg_id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <auto>) ostream)
  "Serializes a message object of type '<auto>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'arm) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'latitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'longitude))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'setstage)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'text))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'text))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'aruco_coordinates))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'aruco_coordinates))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'reference))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'reference))
  (cl:let* ((signed (cl:slot-value msg 'msg_id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <auto>) istream)
  "Deserializes a message object of type '<auto>"
    (cl:setf (cl:slot-value msg 'arm) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'latitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'longitude) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'setstage) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'text) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'text) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'aruco_coordinates) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'aruco_coordinates)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'reference) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'reference) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'msg_id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<auto>)))
  "Returns string type for a message object of type '<auto>"
  "navigation2/auto")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'auto)))
  "Returns string type for a message object of type 'auto"
  "navigation2/auto")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<auto>)))
  "Returns md5sum for a message object of type '<auto>"
  "9fc9b7878baa2559a000819e683c677c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'auto)))
  "Returns md5sum for a message object of type 'auto"
  "9fc9b7878baa2559a000819e683c677c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<auto>)))
  "Returns full string definition for message of type '<auto>"
  (cl:format cl:nil "bool arm~%float32 latitude ~%float32 longitude~%int8 setstage ~%string text~%float32[] aruco_coordinates~%string reference ~%int8 msg_id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'auto)))
  "Returns full string definition for message of type 'auto"
  (cl:format cl:nil "bool arm~%float32 latitude ~%float32 longitude~%int8 setstage ~%string text~%float32[] aruco_coordinates~%string reference ~%int8 msg_id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <auto>))
  (cl:+ 0
     1
     4
     4
     1
     4 (cl:length (cl:slot-value msg 'text))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'aruco_coordinates) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:length (cl:slot-value msg 'reference))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <auto>))
  "Converts a ROS message object to a list"
  (cl:list 'auto
    (cl:cons ':arm (arm msg))
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
    (cl:cons ':setstage (setstage msg))
    (cl:cons ':text (text msg))
    (cl:cons ':aruco_coordinates (aruco_coordinates msg))
    (cl:cons ':reference (reference msg))
    (cl:cons ':msg_id (msg_id msg))
))
