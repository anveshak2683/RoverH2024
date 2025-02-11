; Auto-generated. Do not edit!


(cl:in-package navigation2-msg)


;//! \htmlinclude enc_feed.msg.html

(cl:defclass <enc_feed> (roslisp-msg-protocol:ros-message)
  ((vel
    :reader vel
    :initarg :vel
    :type cl:fixnum
    :initform 0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:fixnum
    :initform 0))
)

(cl:defclass enc_feed (<enc_feed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <enc_feed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'enc_feed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation2-msg:<enc_feed> is deprecated: use navigation2-msg:enc_feed instead.")))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <enc_feed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:vel-val is deprecated.  Use navigation2-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <enc_feed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:angle-val is deprecated.  Use navigation2-msg:angle instead.")
  (angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <enc_feed>) ostream)
  "Serializes a message object of type '<enc_feed>"
  (cl:let* ((signed (cl:slot-value msg 'vel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'angle)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <enc_feed>) istream)
  "Deserializes a message object of type '<enc_feed>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vel) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'angle) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<enc_feed>)))
  "Returns string type for a message object of type '<enc_feed>"
  "navigation2/enc_feed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'enc_feed)))
  "Returns string type for a message object of type 'enc_feed"
  "navigation2/enc_feed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<enc_feed>)))
  "Returns md5sum for a message object of type '<enc_feed>"
  "c7d324c5346393c6bc385a7a281fa92c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'enc_feed)))
  "Returns md5sum for a message object of type 'enc_feed"
  "c7d324c5346393c6bc385a7a281fa92c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<enc_feed>)))
  "Returns full string definition for message of type '<enc_feed>"
  (cl:format cl:nil "int16 vel~%int16 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'enc_feed)))
  "Returns full string definition for message of type 'enc_feed"
  (cl:format cl:nil "int16 vel~%int16 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <enc_feed>))
  (cl:+ 0
     2
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <enc_feed>))
  "Converts a ROS message object to a list"
  (cl:list 'enc_feed
    (cl:cons ':vel (vel msg))
    (cl:cons ':angle (angle msg))
))
