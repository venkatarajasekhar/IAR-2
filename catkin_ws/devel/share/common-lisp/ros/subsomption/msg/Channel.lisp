; Auto-generated. Do not edit!


(cl:in-package subsomption-msg)


;//! \htmlinclude Channel.msg.html

(cl:defclass <Channel> (roslisp-msg-protocol:ros-message)
  ((activated
    :reader activated
    :initarg :activated
    :type cl:boolean
    :initform cl:nil)
   (speed_left
    :reader speed_left
    :initarg :speed_left
    :type cl:float
    :initform 0.0)
   (speed_right
    :reader speed_right
    :initarg :speed_right
    :type cl:float
    :initform 0.0))
)

(cl:defclass Channel (<Channel>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Channel>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Channel)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name subsomption-msg:<Channel> is deprecated: use subsomption-msg:Channel instead.")))

(cl:ensure-generic-function 'activated-val :lambda-list '(m))
(cl:defmethod activated-val ((m <Channel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subsomption-msg:activated-val is deprecated.  Use subsomption-msg:activated instead.")
  (activated m))

(cl:ensure-generic-function 'speed_left-val :lambda-list '(m))
(cl:defmethod speed_left-val ((m <Channel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subsomption-msg:speed_left-val is deprecated.  Use subsomption-msg:speed_left instead.")
  (speed_left m))

(cl:ensure-generic-function 'speed_right-val :lambda-list '(m))
(cl:defmethod speed_right-val ((m <Channel>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader subsomption-msg:speed_right-val is deprecated.  Use subsomption-msg:speed_right instead.")
  (speed_right m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Channel>) ostream)
  "Serializes a message object of type '<Channel>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'activated) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Channel>) istream)
  "Deserializes a message object of type '<Channel>"
    (cl:setf (cl:slot-value msg 'activated) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_left) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed_right) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Channel>)))
  "Returns string type for a message object of type '<Channel>"
  "subsomption/Channel")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Channel)))
  "Returns string type for a message object of type 'Channel"
  "subsomption/Channel")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Channel>)))
  "Returns md5sum for a message object of type '<Channel>"
  "8a78b9051527ab8b87f5963e4ef4b49d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Channel)))
  "Returns md5sum for a message object of type 'Channel"
  "8a78b9051527ab8b87f5963e4ef4b49d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Channel>)))
  "Returns full string definition for message of type '<Channel>"
  (cl:format cl:nil "bool activated~%float32 speed_left~%float32 speed_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Channel)))
  "Returns full string definition for message of type 'Channel"
  (cl:format cl:nil "bool activated~%float32 speed_left~%float32 speed_right~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Channel>))
  (cl:+ 0
     1
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Channel>))
  "Converts a ROS message object to a list"
  (cl:list 'Channel
    (cl:cons ':activated (activated msg))
    (cl:cons ':speed_left (speed_left msg))
    (cl:cons ':speed_right (speed_right msg))
))
