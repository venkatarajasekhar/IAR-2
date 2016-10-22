; Auto-generated. Do not edit!


(cl:in-package fastsim-srv)


;//! \htmlinclude Teleport-request.msg.html

(cl:defclass <Teleport-request> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (theta
    :reader theta
    :initarg :theta
    :type cl:float
    :initform 0.0))
)

(cl:defclass Teleport-request (<Teleport-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Teleport-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Teleport-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fastsim-srv:<Teleport-request> is deprecated: use fastsim-srv:Teleport-request instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Teleport-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastsim-srv:x-val is deprecated.  Use fastsim-srv:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Teleport-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastsim-srv:y-val is deprecated.  Use fastsim-srv:y instead.")
  (y m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <Teleport-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastsim-srv:theta-val is deprecated.  Use fastsim-srv:theta instead.")
  (theta m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Teleport-request>) ostream)
  "Serializes a message object of type '<Teleport-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Teleport-request>) istream)
  "Deserializes a message object of type '<Teleport-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'theta) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Teleport-request>)))
  "Returns string type for a service object of type '<Teleport-request>"
  "fastsim/TeleportRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Teleport-request)))
  "Returns string type for a service object of type 'Teleport-request"
  "fastsim/TeleportRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Teleport-request>)))
  "Returns md5sum for a message object of type '<Teleport-request>"
  "3c45dffe54c7893cd80dfeeae860db74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Teleport-request)))
  "Returns md5sum for a message object of type 'Teleport-request"
  "3c45dffe54c7893cd80dfeeae860db74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Teleport-request>)))
  "Returns full string definition for message of type '<Teleport-request>"
  (cl:format cl:nil "float32 x~%float32 y~%float32 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Teleport-request)))
  "Returns full string definition for message of type 'Teleport-request"
  (cl:format cl:nil "float32 x~%float32 y~%float32 theta~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Teleport-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Teleport-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Teleport-request
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':theta (theta msg))
))
;//! \htmlinclude Teleport-response.msg.html

(cl:defclass <Teleport-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Teleport-response (<Teleport-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Teleport-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Teleport-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fastsim-srv:<Teleport-response> is deprecated: use fastsim-srv:Teleport-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <Teleport-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastsim-srv:ack-val is deprecated.  Use fastsim-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Teleport-response>) ostream)
  "Serializes a message object of type '<Teleport-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ack) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Teleport-response>) istream)
  "Deserializes a message object of type '<Teleport-response>"
    (cl:setf (cl:slot-value msg 'ack) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Teleport-response>)))
  "Returns string type for a service object of type '<Teleport-response>"
  "fastsim/TeleportResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Teleport-response)))
  "Returns string type for a service object of type 'Teleport-response"
  "fastsim/TeleportResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Teleport-response>)))
  "Returns md5sum for a message object of type '<Teleport-response>"
  "3c45dffe54c7893cd80dfeeae860db74")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Teleport-response)))
  "Returns md5sum for a message object of type 'Teleport-response"
  "3c45dffe54c7893cd80dfeeae860db74")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Teleport-response>)))
  "Returns full string definition for message of type '<Teleport-response>"
  (cl:format cl:nil "bool ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Teleport-response)))
  "Returns full string definition for message of type 'Teleport-response"
  (cl:format cl:nil "bool ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Teleport-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Teleport-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Teleport-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Teleport)))
  'Teleport-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Teleport)))
  'Teleport-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Teleport)))
  "Returns string type for a service object of type '<Teleport>"
  "fastsim/Teleport")