; Auto-generated. Do not edit!


(cl:in-package navigation2-msg)


;//! \htmlinclude imu_angle.msg.html

(cl:defclass <imu_angle> (roslisp-msg-protocol:ros-message)
  ((Roll
    :reader Roll
    :initarg :Roll
    :type cl:float
    :initform 0.0)
   (Pitch
    :reader Pitch
    :initarg :Pitch
    :type cl:float
    :initform 0.0)
   (Yaw
    :reader Yaw
    :initarg :Yaw
    :type cl:float
    :initform 0.0))
)

(cl:defclass imu_angle (<imu_angle>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <imu_angle>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'imu_angle)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation2-msg:<imu_angle> is deprecated: use navigation2-msg:imu_angle instead.")))

(cl:ensure-generic-function 'Roll-val :lambda-list '(m))
(cl:defmethod Roll-val ((m <imu_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:Roll-val is deprecated.  Use navigation2-msg:Roll instead.")
  (Roll m))

(cl:ensure-generic-function 'Pitch-val :lambda-list '(m))
(cl:defmethod Pitch-val ((m <imu_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:Pitch-val is deprecated.  Use navigation2-msg:Pitch instead.")
  (Pitch m))

(cl:ensure-generic-function 'Yaw-val :lambda-list '(m))
(cl:defmethod Yaw-val ((m <imu_angle>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:Yaw-val is deprecated.  Use navigation2-msg:Yaw instead.")
  (Yaw m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <imu_angle>) ostream)
  "Serializes a message object of type '<imu_angle>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Roll))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Pitch))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Yaw))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <imu_angle>) istream)
  "Deserializes a message object of type '<imu_angle>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Roll) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Pitch) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Yaw) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<imu_angle>)))
  "Returns string type for a message object of type '<imu_angle>"
  "navigation2/imu_angle")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'imu_angle)))
  "Returns string type for a message object of type 'imu_angle"
  "navigation2/imu_angle")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<imu_angle>)))
  "Returns md5sum for a message object of type '<imu_angle>"
  "d8aec502c207f580f351d8f6036c0af0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'imu_angle)))
  "Returns md5sum for a message object of type 'imu_angle"
  "d8aec502c207f580f351d8f6036c0af0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<imu_angle>)))
  "Returns full string definition for message of type '<imu_angle>"
  (cl:format cl:nil "float32 Roll~%float32 Pitch~%float32 Yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'imu_angle)))
  "Returns full string definition for message of type 'imu_angle"
  (cl:format cl:nil "float32 Roll~%float32 Pitch~%float32 Yaw~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <imu_angle>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <imu_angle>))
  "Converts a ROS message object to a list"
  (cl:list 'imu_angle
    (cl:cons ':Roll (Roll msg))
    (cl:cons ':Pitch (Pitch msg))
    (cl:cons ':Yaw (Yaw msg))
))
