; Auto-generated. Do not edit!


(cl:in-package navigation2-msg)


;//! \htmlinclude gps_data.msg.html

(cl:defclass <gps_data> (roslisp-msg-protocol:ros-message)
  ((latitude
    :reader latitude
    :initarg :latitude
    :type cl:float
    :initform 0.0)
   (longitude
    :reader longitude
    :initarg :longitude
    :type cl:float
    :initform 0.0))
)

(cl:defclass gps_data (<gps_data>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <gps_data>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'gps_data)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation2-msg:<gps_data> is deprecated: use navigation2-msg:gps_data instead.")))

(cl:ensure-generic-function 'latitude-val :lambda-list '(m))
(cl:defmethod latitude-val ((m <gps_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:latitude-val is deprecated.  Use navigation2-msg:latitude instead.")
  (latitude m))

(cl:ensure-generic-function 'longitude-val :lambda-list '(m))
(cl:defmethod longitude-val ((m <gps_data>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:longitude-val is deprecated.  Use navigation2-msg:longitude instead.")
  (longitude m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <gps_data>) ostream)
  "Serializes a message object of type '<gps_data>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <gps_data>) istream)
  "Deserializes a message object of type '<gps_data>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<gps_data>)))
  "Returns string type for a message object of type '<gps_data>"
  "navigation2/gps_data")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'gps_data)))
  "Returns string type for a message object of type 'gps_data"
  "navigation2/gps_data")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<gps_data>)))
  "Returns md5sum for a message object of type '<gps_data>"
  "9aeb2245d9611f300beeb62a0151d3f3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'gps_data)))
  "Returns md5sum for a message object of type 'gps_data"
  "9aeb2245d9611f300beeb62a0151d3f3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<gps_data>)))
  "Returns full string definition for message of type '<gps_data>"
  (cl:format cl:nil "float32 latitude~%float32 longitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'gps_data)))
  "Returns full string definition for message of type 'gps_data"
  (cl:format cl:nil "float32 latitude~%float32 longitude~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <gps_data>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <gps_data>))
  "Converts a ROS message object to a list"
  (cl:list 'gps_data
    (cl:cons ':latitude (latitude msg))
    (cl:cons ':longitude (longitude msg))
))
