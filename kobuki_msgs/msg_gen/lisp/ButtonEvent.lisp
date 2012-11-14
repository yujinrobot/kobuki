; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude ButtonEvent.msg.html

(cl:defclass <ButtonEvent> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0)
   (button
    :reader button
    :initarg :button
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ButtonEvent (<ButtonEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ButtonEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ButtonEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<ButtonEvent> is deprecated: use kobuki_msgs-msg:ButtonEvent instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <ButtonEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:state-val is deprecated.  Use kobuki_msgs-msg:state instead.")
  (state m))

(cl:ensure-generic-function 'button-val :lambda-list '(m))
(cl:defmethod button-val ((m <ButtonEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:button-val is deprecated.  Use kobuki_msgs-msg:button instead.")
  (button m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<ButtonEvent>)))
    "Constants for message type '<ButtonEvent>"
  '((:BUTTON0 . 0)
    (:BUTTON1 . 1)
    (:BUTTON2 . 2)
    (:RELEASED . 0)
    (:PRESSED . 1))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'ButtonEvent)))
    "Constants for message type 'ButtonEvent"
  '((:BUTTON0 . 0)
    (:BUTTON1 . 1)
    (:BUTTON2 . 2)
    (:RELEASED . 0)
    (:PRESSED . 1))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ButtonEvent>) ostream)
  "Serializes a message object of type '<ButtonEvent>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'button)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ButtonEvent>) istream)
  "Deserializes a message object of type '<ButtonEvent>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'state)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'button)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ButtonEvent>)))
  "Returns string type for a message object of type '<ButtonEvent>"
  "kobuki_msgs/ButtonEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ButtonEvent)))
  "Returns string type for a message object of type 'ButtonEvent"
  "kobuki_msgs/ButtonEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ButtonEvent>)))
  "Returns md5sum for a message object of type '<ButtonEvent>"
  "627fdd6fd6471539e5190c0e4fadb9c2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ButtonEvent)))
  "Returns md5sum for a message object of type 'ButtonEvent"
  "627fdd6fd6471539e5190c0e4fadb9c2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ButtonEvent>)))
  "Returns full string definition for message of type '<ButtonEvent>"
  (cl:format cl:nil "# Provides a button event~%# This message is generated whenever a button is pressed or released~%~%uint8 Button0 = 0~%uint8 Button1 = 1~%uint8 Button2 = 2~%~%uint8 RELEASED = 0~%uint8 PRESSED  = 1~%~%uint8 state~%uint8 button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ButtonEvent)))
  "Returns full string definition for message of type 'ButtonEvent"
  (cl:format cl:nil "# Provides a button event~%# This message is generated whenever a button is pressed or released~%~%uint8 Button0 = 0~%uint8 Button1 = 1~%uint8 Button2 = 2~%~%uint8 RELEASED = 0~%uint8 PRESSED  = 1~%~%uint8 state~%uint8 button~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ButtonEvent>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ButtonEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'ButtonEvent
    (cl:cons ':state (state msg))
    (cl:cons ':button (button msg))
))
