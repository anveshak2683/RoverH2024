; Auto-generated. Do not edit!


(cl:in-package navigation2-msg)


;//! \htmlinclude detection.msg.html

(cl:defclass <detection> (roslisp-msg-protocol:ros-message)
  ((color
    :reader color
    :initarg :color
    :type cl:string
    :initform "")
   (depth
    :reader depth
    :initarg :depth
    :type cl:float
    :initform 0.0))
)

(cl:defclass detection (<detection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <detection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'detection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation2-msg:<detection> is deprecated: use navigation2-msg:detection instead.")))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:color-val is deprecated.  Use navigation2-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'depth-val :lambda-list '(m))
(cl:defmethod depth-val ((m <detection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:depth-val is deprecated.  Use navigation2-msg:depth instead.")
  (depth m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <detection>) ostream)
  "Serializes a message object of type '<detection>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'color))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'depth))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <detection>) istream)
  "Deserializes a message object of type '<detection>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'color) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'depth) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<detection>)))
  "Returns string type for a message object of type '<detection>"
  "navigation2/detection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'detection)))
  "Returns string type for a message object of type 'detection"
  "navigation2/detection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<detection>)))
  "Returns md5sum for a message object of type '<detection>"
  "54cdd292028a384f7f8301d873bb5638")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'detection)))
  "Returns md5sum for a message object of type 'detection"
  "54cdd292028a384f7f8301d873bb5638")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<detection>)))
  "Returns full string definition for message of type '<detection>"
  (cl:format cl:nil "string color~%float32 depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'detection)))
  "Returns full string definition for message of type 'detection"
  (cl:format cl:nil "string color~%float32 depth~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <detection>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'color))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <detection>))
  "Converts a ROS message object to a list"
  (cl:list 'detection
    (cl:cons ':color (color msg))
    (cl:cons ':depth (depth msg))
))
