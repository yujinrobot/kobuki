; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude CliffEvent.msg.html

(cl:defclass <CliffEvent> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (sensor
    :reader sensor
    :initarg :sensor
    :type cl:fixnum
    :initform 0)
   (bottom
    :reader bottom
    :initarg :bottom
    :type cl:fixnum
    :initform 0))
)

(cl:defclass CliffEvent (<CliffEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CliffEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CliffEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<CliffEvent> is deprecated: use kobuki_msgs-msg:CliffEvent instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <CliffEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:state-val is deprecated.  Use kobuki_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'sensor-val :lambda-list '(m))
(cl:defmethod sensor-val ((m <CliffEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:sensor-val is deprecated.  Use kobuki_msgs-msg:sensor instead.")
  (sensor m))

(cl:ensure-generic-function 'bottom-val :lambda-list '(m))
(cl:defmethod bottom-val ((m <CliffEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:bottom-val is deprecated.  Use kobuki_msgs-msg:bottom instead.")
  (bottom m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<CliffEvent>)))
    "Constants for message type '<CliffEvent>"
  '((:LEFT . 0)
    (:CENTER . 1)
    (:RIGHT . 2)
    (:FLOOR . 0)
    (:CLIFF . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'CliffEvent)))
    "Constants for message type 'CliffEvent"
  '((:LEFT . 0)
    (:CENTER . 1)
    (:RIGHT . 2)
    (:FLOOR . 0)
    (:CLIFF . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CliffEvent>) ostream)
  "Serializes a message object of type '<CliffEvent>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bottom)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bottom)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CliffEvent>) istream)
  "Deserializes a message object of type '<CliffEvent>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'sensor)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'bottom)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'bottom)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CliffEvent>)))
  "Returns string type for a message object of type '<CliffEvent>"
  "kobuki_msgs/CliffEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CliffEvent)))
  "Returns string type for a message object of type 'CliffEvent"
  "kobuki_msgs/CliffEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CliffEvent>)))
  "Returns md5sum for a message object of type '<CliffEvent>"
  "768947926ed252fd64a5711160ff3884")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CliffEvent)))
  "Returns md5sum for a message object of type 'CliffEvent"
  "768947926ed252fd64a5711160ff3884")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CliffEvent>)))
  "Returns full string definition for message of type '<CliffEvent>"
  (cl:format cl:nil "# Provides a cliff sensor event~%# This message is generated whenever the robot approaches or moves away from a cliff~%~%uint8 LEFT   = 0~%uint8 CENTER = 1~%uint8 RIGHT  = 2~%~%uint8 FLOOR = 0~%uint8 CLIFF = 1~%~%uint8  state~%uint8  sensor~%uint16 bottom~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CliffEvent)))
  "Returns full string definition for message of type 'CliffEvent"
  (cl:format cl:nil "# Provides a cliff sensor event~%# This message is generated whenever the robot approaches or moves away from a cliff~%~%uint8 LEFT   = 0~%uint8 CENTER = 1~%uint8 RIGHT  = 2~%~%uint8 FLOOR = 0~%uint8 CLIFF = 1~%~%uint8  state~%uint8  sensor~%uint16 bottom~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CliffEvent>))
  (cl:+ 0
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CliffEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'CliffEvent
    (cl:cons ':state (state msg))
    (cl:cons ':sensor (sensor msg))
    (cl:cons ':bottom (bottom msg))
))
