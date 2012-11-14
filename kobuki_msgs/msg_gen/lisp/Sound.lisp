; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude Sound.msg.html

(cl:defclass <Sound> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Sound (<Sound>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Sound>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Sound)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<Sound> is deprecated: use kobuki_msgs-msg:Sound instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Sound>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:value-val is deprecated.  Use kobuki_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<Sound>)))
    "Constants for message type '<Sound>"
  '((:ON . 0)
    (:OFF . 1)
    (:RECHARGE . 2)
    (:BUTTON . 3)
    (:ERROR . 4)
    (:CLEANINGSTART . 5)
    (:CLEANINGEND . 6))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'Sound)))
    "Constants for message type 'Sound"
  '((:ON . 0)
    (:OFF . 1)
    (:RECHARGE . 2)
    (:BUTTON . 3)
    (:ERROR . 4)
    (:CLEANINGSTART . 5)
    (:CLEANINGEND . 6))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Sound>) ostream)
  "Serializes a message object of type '<Sound>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Sound>) istream)
  "Deserializes a message object of type '<Sound>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'value)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Sound>)))
  "Returns string type for a message object of type '<Sound>"
  "kobuki_msgs/Sound")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Sound)))
  "Returns string type for a message object of type 'Sound"
  "kobuki_msgs/Sound")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Sound>)))
  "Returns md5sum for a message object of type '<Sound>"
  "dfeab0daae67749c426c1db741a4f420")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Sound)))
  "Returns md5sum for a message object of type 'Sound"
  "dfeab0daae67749c426c1db741a4f420")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Sound>)))
  "Returns full string definition for message of type '<Sound>"
  (cl:format cl:nil "# Sends a command for playing sounds.~%# The available sound sequences:~%# 0 - turn on~%# 1 - turn off~%# 2 - recharge start~%# 3 - press button,~%# 4 - error sound~%# 5 - start cleaning~%# 6 - cleaning end~%~%uint8 ON            = 0~%uint8 OFF           = 1~%uint8 RECHARGE      = 2~%uint8 BUTTON        = 3~%uint8 ERROR         = 4~%uint8 CLEANINGSTART = 5~%uint8 CLEANINGEND   = 6~%~%uint8 value~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Sound)))
  "Returns full string definition for message of type 'Sound"
  (cl:format cl:nil "# Sends a command for playing sounds.~%# The available sound sequences:~%# 0 - turn on~%# 1 - turn off~%# 2 - recharge start~%# 3 - press button,~%# 4 - error sound~%# 5 - start cleaning~%# 6 - cleaning end~%~%uint8 ON            = 0~%uint8 OFF           = 1~%uint8 RECHARGE      = 2~%uint8 BUTTON        = 3~%uint8 ERROR         = 4~%uint8 CLEANINGSTART = 5~%uint8 CLEANINGEND   = 6~%~%uint8 value~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Sound>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Sound>))
  "Converts a ROS message object to a list"
  (cl:list 'Sound
    (cl:cons ':value (value msg))
))
