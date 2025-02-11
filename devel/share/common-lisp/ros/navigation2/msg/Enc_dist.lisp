; Auto-generated. Do not edit!


(cl:in-package navigation2-msg)


;//! \htmlinclude Enc_dist.msg.html

(cl:defclass <Enc_dist> (roslisp-msg-protocol:ros-message)
  ((dist
    :reader dist
    :initarg :dist
    :type cl:float
    :initform 0.0))
)

(cl:defclass Enc_dist (<Enc_dist>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Enc_dist>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Enc_dist)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation2-msg:<Enc_dist> is deprecated: use navigation2-msg:Enc_dist instead.")))

(cl:ensure-generic-function 'dist-val :lambda-list '(m))
(cl:defmethod dist-val ((m <Enc_dist>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation2-msg:dist-val is deprecated.  Use navigation2-msg:dist instead.")
  (dist m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Enc_dist>) ostream)
  "Serializes a message object of type '<Enc_dist>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'dist))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Enc_dist>) istream)
  "Deserializes a message object of type '<Enc_dist>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'dist) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Enc_dist>)))
  "Returns string type for a message object of type '<Enc_dist>"
  "navigation2/Enc_dist")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Enc_dist)))
  "Returns string type for a message object of type 'Enc_dist"
  "navigation2/Enc_dist")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Enc_dist>)))
  "Returns md5sum for a message object of type '<Enc_dist>"
  "3f4fece6412db25720b2bab9f80b3473")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Enc_dist)))
  "Returns md5sum for a message object of type 'Enc_dist"
  "3f4fece6412db25720b2bab9f80b3473")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Enc_dist>)))
  "Returns full string definition for message of type '<Enc_dist>"
  (cl:format cl:nil "float64 dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Enc_dist)))
  "Returns full string definition for message of type 'Enc_dist"
  (cl:format cl:nil "float64 dist~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Enc_dist>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Enc_dist>))
  "Converts a ROS message object to a list"
  (cl:list 'Enc_dist
    (cl:cons ':dist (dist msg))
))
