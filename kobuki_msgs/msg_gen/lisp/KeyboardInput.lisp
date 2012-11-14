; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude KeyboardInput.msg.html

(cl:defclass <KeyboardInput> (roslisp-msg-protocol:ros-message)
  ((pressedKey
    :reader pressedKey
    :initarg :pressedKey
    :type cl:fixnum
    :initform 0))
)

(cl:defclass KeyboardInput (<KeyboardInput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <KeyboardInput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'KeyboardInput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<KeyboardInput> is deprecated: use kobuki_msgs-msg:KeyboardInput instead.")))

(cl:ensure-generic-function 'pressedKey-val :lambda-list '(m))
(cl:defmethod pressedKey-val ((m <KeyboardInput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:pressedKey-val is deprecated.  Use kobuki_msgs-msg:pressedKey instead.")
  (pressedKey m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<KeyboardInput>)))
    "Constants for message type '<KeyboardInput>"
  '((:KEYCODE_RIGHT . 67)
    (:KEYCODE_LEFT . 68)
    (:KEYCODE_UP . 65)
    (:KEYCODE_DOWN . 66)
    (:KEYCODE_SPACE . 32)
    (:KEYCODE_ENABLE . 101)
    (:KEYCODE_DISABLE . 100))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'KeyboardInput)))
    "Constants for message type 'KeyboardInput"
  '((:KEYCODE_RIGHT . 67)
    (:KEYCODE_LEFT . 68)
    (:KEYCODE_UP . 65)
    (:KEYCODE_DOWN . 66)
    (:KEYCODE_SPACE . 32)
    (:KEYCODE_ENABLE . 101)
    (:KEYCODE_DISABLE . 100))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <KeyboardInput>) ostream)
  "Serializes a message object of type '<KeyboardInput>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pressedKey)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <KeyboardInput>) istream)
  "Deserializes a message object of type '<KeyboardInput>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'pressedKey)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<KeyboardInput>)))
  "Returns string type for a message object of type '<KeyboardInput>"
  "kobuki_msgs/KeyboardInput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'KeyboardInput)))
  "Returns string type for a message object of type 'KeyboardInput"
  "kobuki_msgs/KeyboardInput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<KeyboardInput>)))
  "Returns md5sum for a message object of type '<KeyboardInput>"
  "9288b95cb32b48719d84d696be253401")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'KeyboardInput)))
  "Returns md5sum for a message object of type 'KeyboardInput"
  "9288b95cb32b48719d84d696be253401")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<KeyboardInput>)))
  "Returns full string definition for message of type '<KeyboardInput>"
  (cl:format cl:nil "# KEYBOARD INPUT~%# ~%# Keycodes to be transferred for remote teleops.~%~%uint8  KeyCode_Right    = 67     # 0x43~%uint8  KeyCode_Left     = 68     # 0x44~%uint8  KeyCode_Up       = 65     # 0x41~%uint8  KeyCode_Down     = 66     # 0x42~%uint8  KeyCode_Space    = 32     # 0x20~%uint8  KeyCode_Enable   = 101    # 0x65, 'e'~%uint8  KeyCode_Disable  = 100    # 0x64, 'd'~%~%uint8 pressedKey~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'KeyboardInput)))
  "Returns full string definition for message of type 'KeyboardInput"
  (cl:format cl:nil "# KEYBOARD INPUT~%# ~%# Keycodes to be transferred for remote teleops.~%~%uint8  KeyCode_Right    = 67     # 0x43~%uint8  KeyCode_Left     = 68     # 0x44~%uint8  KeyCode_Up       = 65     # 0x41~%uint8  KeyCode_Down     = 66     # 0x42~%uint8  KeyCode_Space    = 32     # 0x20~%uint8  KeyCode_Enable   = 101    # 0x65, 'e'~%uint8  KeyCode_Disable  = 100    # 0x64, 'd'~%~%uint8 pressedKey~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <KeyboardInput>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <KeyboardInput>))
  "Converts a ROS message object to a list"
  (cl:list 'KeyboardInput
    (cl:cons ':pressedKey (pressedKey msg))
))
