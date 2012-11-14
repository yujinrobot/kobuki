; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude RobotStateEvent.msg.html

(cl:defclass <RobotStateEvent> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass RobotStateEvent (<RobotStateEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <RobotStateEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'RobotStateEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<RobotStateEvent> is deprecated: use kobuki_msgs-msg:RobotStateEvent instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <RobotStateEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:state-val is deprecated.  Use kobuki_msgs-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<RobotStateEvent>)))
    "Constants for message type '<RobotStateEvent>"
  '((:ONLINE . 1)
    (:OFFLINE . 0))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'RobotStateEvent)))
    "Constants for message type 'RobotStateEvent"
  '((:ONLINE . 1)
    (:OFFLINE . 0))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <RobotStateEvent>) ostream)
  "Serializes a message object of type '<RobotStateEvent>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <RobotStateEvent>) istream)
  "Deserializes a message object of type '<RobotStateEvent>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<RobotStateEvent>)))
  "Returns string type for a message object of type '<RobotStateEvent>"
  "kobuki_msgs/RobotStateEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'RobotStateEvent)))
  "Returns string type for a message object of type 'RobotStateEvent"
  "kobuki_msgs/RobotStateEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<RobotStateEvent>)))
  "Returns md5sum for a message object of type '<RobotStateEvent>"
  "c6eccd4cb1f95df95635b56d6226ea32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'RobotStateEvent)))
  "Returns md5sum for a message object of type 'RobotStateEvent"
  "c6eccd4cb1f95df95635b56d6226ea32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<RobotStateEvent>)))
  "Returns full string definition for message of type '<RobotStateEvent>"
  (cl:format cl:nil "# Provides a robot state event~%# This message is generated whenever the robot gets online/offline~%~%uint8 ONLINE  = 1~%uint8 OFFLINE = 0~%~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'RobotStateEvent)))
  "Returns full string definition for message of type 'RobotStateEvent"
  (cl:format cl:nil "# Provides a robot state event~%# This message is generated whenever the robot gets online/offline~%~%uint8 ONLINE  = 1~%uint8 OFFLINE = 0~%~%uint8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <RobotStateEvent>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <RobotStateEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'RobotStateEvent
    (cl:cons ':state (state msg))
))
