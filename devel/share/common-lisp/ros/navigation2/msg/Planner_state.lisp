; Auto-generated. Do not edit!


(cl:in-package navigation2-msg)


;//! \htmlinclude Planner_state.msg.html

(cl:defclass <Planner_state> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Planner_state (<Planner_state>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Planner_state>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Planner_state)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation2-msg:<Planner_state> is deprecated: use navigation2-msg:Planner_state instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <Planner_state>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:status-val is deprecated.  Use navigation2-msg:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Planner_state>) ostream)
  "Serializes a message object of type '<Planner_state>"
  (cl:let* ((signed (cl:slot-value msg 'status)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Planner_state>) istream)
  "Deserializes a message object of type '<Planner_state>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'status) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Planner_state>)))
  "Returns string type for a message object of type '<Planner_state>"
  "navigation2/Planner_state")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Planner_state)))
  "Returns string type for a message object of type 'Planner_state"
  "navigation2/Planner_state")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Planner_state>)))
  "Returns md5sum for a message object of type '<Planner_state>"
  "46fcc1fee3807f5925730339e5177777")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Planner_state)))
  "Returns md5sum for a message object of type 'Planner_state"
  "46fcc1fee3807f5925730339e5177777")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Planner_state>)))
  "Returns full string definition for message of type '<Planner_state>"
  (cl:format cl:nil "int16 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Planner_state)))
  "Returns full string definition for message of type 'Planner_state"
  (cl:format cl:nil "int16 status~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Planner_state>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Planner_state>))
  "Converts a ROS message object to a list"
  (cl:list 'Planner_state
    (cl:cons ':status (status msg))
))
