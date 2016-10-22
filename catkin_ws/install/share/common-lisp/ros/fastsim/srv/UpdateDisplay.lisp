; Auto-generated. Do not edit!


(cl:in-package fastsim-srv)


;//! \htmlinclude UpdateDisplay-request.msg.html

(cl:defclass <UpdateDisplay-request> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateDisplay-request (<UpdateDisplay-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateDisplay-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateDisplay-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fastsim-srv:<UpdateDisplay-request> is deprecated: use fastsim-srv:UpdateDisplay-request instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <UpdateDisplay-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastsim-srv:state-val is deprecated.  Use fastsim-srv:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateDisplay-request>) ostream)
  "Serializes a message object of type '<UpdateDisplay-request>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'state) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateDisplay-request>) istream)
  "Deserializes a message object of type '<UpdateDisplay-request>"
    (cl:setf (cl:slot-value msg 'state) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateDisplay-request>)))
  "Returns string type for a service object of type '<UpdateDisplay-request>"
  "fastsim/UpdateDisplayRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateDisplay-request)))
  "Returns string type for a service object of type 'UpdateDisplay-request"
  "fastsim/UpdateDisplayRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateDisplay-request>)))
  "Returns md5sum for a message object of type '<UpdateDisplay-request>"
  "ff528e8c361f0e5edec96e78493d753f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateDisplay-request)))
  "Returns md5sum for a message object of type 'UpdateDisplay-request"
  "ff528e8c361f0e5edec96e78493d753f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateDisplay-request>)))
  "Returns full string definition for message of type '<UpdateDisplay-request>"
  (cl:format cl:nil "bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateDisplay-request)))
  "Returns full string definition for message of type 'UpdateDisplay-request"
  (cl:format cl:nil "bool state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateDisplay-request>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateDisplay-request>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateDisplay-request
    (cl:cons ':state (state msg))
))
;//! \htmlinclude UpdateDisplay-response.msg.html

(cl:defclass <UpdateDisplay-response> (roslisp-msg-protocol:ros-message)
  ((ack
    :reader ack
    :initarg :ack
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass UpdateDisplay-response (<UpdateDisplay-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <UpdateDisplay-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'UpdateDisplay-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name fastsim-srv:<UpdateDisplay-response> is deprecated: use fastsim-srv:UpdateDisplay-response instead.")))

(cl:ensure-generic-function 'ack-val :lambda-list '(m))
(cl:defmethod ack-val ((m <UpdateDisplay-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader fastsim-srv:ack-val is deprecated.  Use fastsim-srv:ack instead.")
  (ack m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <UpdateDisplay-response>) ostream)
  "Serializes a message object of type '<UpdateDisplay-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'ack) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <UpdateDisplay-response>) istream)
  "Deserializes a message object of type '<UpdateDisplay-response>"
    (cl:setf (cl:slot-value msg 'ack) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<UpdateDisplay-response>)))
  "Returns string type for a service object of type '<UpdateDisplay-response>"
  "fastsim/UpdateDisplayResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateDisplay-response)))
  "Returns string type for a service object of type 'UpdateDisplay-response"
  "fastsim/UpdateDisplayResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<UpdateDisplay-response>)))
  "Returns md5sum for a message object of type '<UpdateDisplay-response>"
  "ff528e8c361f0e5edec96e78493d753f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'UpdateDisplay-response)))
  "Returns md5sum for a message object of type 'UpdateDisplay-response"
  "ff528e8c361f0e5edec96e78493d753f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<UpdateDisplay-response>)))
  "Returns full string definition for message of type '<UpdateDisplay-response>"
  (cl:format cl:nil "bool ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'UpdateDisplay-response)))
  "Returns full string definition for message of type 'UpdateDisplay-response"
  (cl:format cl:nil "bool ack~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <UpdateDisplay-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <UpdateDisplay-response>))
  "Converts a ROS message object to a list"
  (cl:list 'UpdateDisplay-response
    (cl:cons ':ack (ack msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'UpdateDisplay)))
  'UpdateDisplay-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'UpdateDisplay)))
  'UpdateDisplay-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'UpdateDisplay)))
  "Returns string type for a service object of type '<UpdateDisplay>"
  "fastsim/UpdateDisplay")