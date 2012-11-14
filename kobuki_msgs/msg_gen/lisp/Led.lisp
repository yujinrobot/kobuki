; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude Led.msg.html

(cl:defclass <Led> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Led (<Led>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Led>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Led)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<Led> is deprecated: use kobuki_msgs-msg:Led instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Led>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:value-val is deprecated.  Use kobuki_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Led>)))
    "Constants for message type '<Led>"
  '((:BLACK . 0)
    (:GREEN . 1)
    (:ORANGE . 2)
    (:RED . 3))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Led)))
    "Constants for message type 'Led"
  '((:BLACK . 0)
    (:GREEN . 1)
    (:ORANGE . 2)
    (:RED . 3))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Led>) ostream)
  "Serializes a message object of type '<Led>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Led>) istream)
  "Deserializes a message object of type '<Led>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Led>)))
  "Returns string type for a message object of type '<Led>"
  "kobuki_msgs/Led")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Led)))
  "Returns string type for a message object of type 'Led"
  "kobuki_msgs/Led")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Led>)))
  "Returns md5sum for a message object of type '<Led>"
  "4391183b0cf05f8f25d04220401b9f43")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Led)))
  "Returns md5sum for a message object of type 'Led"
  "4391183b0cf05f8f25d04220401b9f43")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Led>)))
  "Returns full string definition for message of type '<Led>"
  (cl:format cl:nil "# Sends a command for controlling the a LED.~%# ~%# Typically the first LED is always reserved to denote~%# the state - the remainder will be controllable. ~%~%uint8 BLACK   = 0~%uint8 GREEN   = 1~%uint8 ORANGE  = 2~%uint8 RED     = 3~%~%# For kobuki there are only two controllable LED's.~%uint8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Led)))
  "Returns full string definition for message of type 'Led"
  (cl:format cl:nil "# Sends a command for controlling the a LED.~%# ~%# Typically the first LED is always reserved to denote~%# the state - the remainder will be controllable. ~%~%uint8 BLACK   = 0~%uint8 GREEN   = 1~%uint8 ORANGE  = 2~%uint8 RED     = 3~%~%# For kobuki there are only two controllable LED's.~%uint8 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Led>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Led>))
  "Converts a ROS message object to a list"
  (cl:list 'Led
    (cl:cons ':value (value msg))
))
