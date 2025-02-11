; Auto-generated. Do not edit!


(cl:in-package navigation2-msg)


;//! \htmlinclude red.msg.html

(cl:defclass <red> (roslisp-msg-protocol:ros-message)
  ((vel
    :reader vel
    :initarg :vel
    :type cl:fixnum
    :initform 0)
   (omega
    :reader omega
    :initarg :omega
    :type cl:fixnum
    :initform 0)
   (detect
    :reader detect
    :initarg :detect
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass red (<red>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <red>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'red)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation2-msg:<red> is deprecated: use navigation2-msg:red instead.")))

(cl:ensure-generic-function 'vel-val :lambda-list '(m))
(cl:defmethod vel-val ((m <red>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:vel-val is deprecated.  Use navigation2-msg:vel instead.")
  (vel m))

(cl:ensure-generic-function 'omega-val :lambda-list '(m))
(cl:defmethod omega-val ((m <red>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:omega-val is deprecated.  Use navigation2-msg:omega instead.")
  (omega m))

(cl:ensure-generic-function 'detect-val :lambda-list '(m))
(cl:defmethod detect-val ((m <red>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:detect-val is deprecated.  Use navigation2-msg:detect instead.")
  (detect m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <red>) ostream)
  "Serializes a message object of type '<red>"
  (cl:let* ((signed (cl:slot-value msg 'vel)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'omega)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'detect) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <red>) istream)
  "Deserializes a message object of type '<red>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'vel) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'omega) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:setf (cl:slot-value msg 'detect) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<red>)))
  "Returns string type for a message object of type '<red>"
  "navigation2/red")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'red)))
  "Returns string type for a message object of type 'red"
  "navigation2/red")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<red>)))
  "Returns md5sum for a message object of type '<red>"
  "598737a047b4631c88d712e78758247d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'red)))
  "Returns md5sum for a message object of type 'red"
  "598737a047b4631c88d712e78758247d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<red>)))
  "Returns full string definition for message of type '<red>"
  (cl:format cl:nil "int16 vel~%int16 omega~%bool detect~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'red)))
  "Returns full string definition for message of type 'red"
  (cl:format cl:nil "int16 vel~%int16 omega~%bool detect~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <red>))
  (cl:+ 0
     2
     2
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <red>))
  "Converts a ROS message object to a list"
  (cl:list 'red
    (cl:cons ':vel (vel msg))
    (cl:cons ':omega (omega msg))
    (cl:cons ':detect (detect msg))
))
