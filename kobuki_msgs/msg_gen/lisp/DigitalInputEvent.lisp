; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude DigitalInputEvent.msg.html

(cl:defclass <DigitalInputEvent> (roslisp-msg-protocol:ros-message)
  ((values
    :reader values
    :initarg :values
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass DigitalInputEvent (<DigitalInputEvent>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DigitalInputEvent>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DigitalInputEvent)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<DigitalInputEvent> is deprecated: use kobuki_msgs-msg:DigitalInputEvent instead.")))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <DigitalInputEvent>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:values-val is deprecated.  Use kobuki_msgs-msg:values instead.")
  (values m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DigitalInputEvent>) ostream)
  "Serializes a message object of type '<DigitalInputEvent>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'values))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DigitalInputEvent>) istream)
  "Deserializes a message object of type '<DigitalInputEvent>"
  (cl:setf (cl:slot-value msg 'values) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'values)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DigitalInputEvent>)))
  "Returns string type for a message object of type '<DigitalInputEvent>"
  "kobuki_msgs/DigitalInputEvent")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalInputEvent)))
  "Returns string type for a message object of type 'DigitalInputEvent"
  "kobuki_msgs/DigitalInputEvent")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DigitalInputEvent>)))
  "Returns md5sum for a message object of type '<DigitalInputEvent>"
  "93da823c8b121f8a3940ef3830c58e44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DigitalInputEvent)))
  "Returns md5sum for a message object of type 'DigitalInputEvent"
  "93da823c8b121f8a3940ef3830c58e44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DigitalInputEvent>)))
  "Returns full string definition for message of type '<DigitalInputEvent>"
  (cl:format cl:nil "# Digital input - only four pins. ~%~%# Array of values indices represent pins 0-3 respectively.~%bool[4] values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DigitalInputEvent)))
  "Returns full string definition for message of type 'DigitalInputEvent"
  (cl:format cl:nil "# Digital input - only four pins. ~%~%# Array of values indices represent pins 0-3 respectively.~%bool[4] values~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DigitalInputEvent>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DigitalInputEvent>))
  "Converts a ROS message object to a list"
  (cl:list 'DigitalInputEvent
    (cl:cons ':values (values msg))
))
