; Auto-generated. Do not edit!


(cl:in-package navigation_strategies-msg)


;//! \htmlinclude DirDistrib.msg.html

(cl:defclass <DirDistrib> (roslisp-msg-protocol:ros-message)
  ((dir
    :reader dir
    :initarg :dir
    :type (cl:vector cl:float)
   :initform (cl:make-array 36 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass DirDistrib (<DirDistrib>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DirDistrib>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DirDistrib)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name navigation_strategies-msg:<DirDistrib> is deprecated: use navigation_strategies-msg:DirDistrib instead.")))

(cl:ensure-generic-function 'dir-val :lambda-list '(m))
(cl:defmethod dir-val ((m <DirDistrib>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader navigation_strategies-msg:dir-val is deprecated.  Use navigation_strategies-msg:dir instead.")
  (dir m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DirDistrib>) ostream)
  "Serializes a message object of type '<DirDistrib>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'dir))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DirDistrib>) istream)
  "Deserializes a message object of type '<DirDistrib>"
  (cl:setf (cl:slot-value msg 'dir) (cl:make-array 36))
  (cl:let ((vals (cl:slot-value msg 'dir)))
    (cl:dotimes (i 36)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DirDistrib>)))
  "Returns string type for a message object of type '<DirDistrib>"
  "navigation_strategies/DirDistrib")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DirDistrib)))
  "Returns string type for a message object of type 'DirDistrib"
  "navigation_strategies/DirDistrib")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DirDistrib>)))
  "Returns md5sum for a message object of type '<DirDistrib>"
  "2b69ee0d0086e7d2ef255f0151b5f20d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DirDistrib)))
  "Returns md5sum for a message object of type 'DirDistrib"
  "2b69ee0d0086e7d2ef255f0151b5f20d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DirDistrib>)))
  "Returns full string definition for message of type '<DirDistrib>"
  (cl:format cl:nil "float64[36] dir~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DirDistrib)))
  "Returns full string definition for message of type 'DirDistrib"
  (cl:format cl:nil "float64[36] dir~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DirDistrib>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'dir) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DirDistrib>))
  "Converts a ROS message object to a list"
  (cl:list 'DirDistrib
    (cl:cons ':dir (dir msg))
))
