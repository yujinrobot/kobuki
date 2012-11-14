; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude DigitalOutput.msg.html

(cl:defclass <DigitalOutput> (roslisp-msg-protocol:ros-message)
  ((values
    :reader values
    :initarg :values
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil))
   (mask
    :reader mask
    :initarg :mask
    :type (cl:vector cl:boolean)
   :initform (cl:make-array 4 :element-type 'cl:boolean :initial-element cl:nil)))
)

(cl:defclass DigitalOutput (<DigitalOutput>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DigitalOutput>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DigitalOutput)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<DigitalOutput> is deprecated: use kobuki_msgs-msg:DigitalOutput instead.")))

(cl:ensure-generic-function 'values-val :lambda-list '(m))
(cl:defmethod values-val ((m <DigitalOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:values-val is deprecated.  Use kobuki_msgs-msg:values instead.")
  (values m))

(cl:ensure-generic-function 'mask-val :lambda-list '(m))
(cl:defmethod mask-val ((m <DigitalOutput>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:mask-val is deprecated.  Use kobuki_msgs-msg:mask instead.")
  (mask m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DigitalOutput>) ostream)
  "Serializes a message object of type '<DigitalOutput>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'values))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if ele 1 0)) ostream))
   (cl:slot-value msg 'mask))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DigitalOutput>) istream)
  "Deserializes a message object of type '<DigitalOutput>"
  (cl:setf (cl:slot-value msg 'values) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'values)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  (cl:setf (cl:slot-value msg 'mask) (cl:make-array 4))
  (cl:let ((vals (cl:slot-value msg 'mask)))
    (cl:dotimes (i 4)
    (cl:setf (cl:aref vals i) (cl:not (cl:zerop (cl:read-byte istream))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DigitalOutput>)))
  "Returns string type for a message object of type '<DigitalOutput>"
  "kobuki_msgs/DigitalOutput")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DigitalOutput)))
  "Returns string type for a message object of type 'DigitalOutput"
  "kobuki_msgs/DigitalOutput")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DigitalOutput>)))
  "Returns md5sum for a message object of type '<DigitalOutput>"
  "1e97cd7f196a0270516b506e8bf29ca2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DigitalOutput)))
  "Returns md5sum for a message object of type 'DigitalOutput"
  "1e97cd7f196a0270516b506e8bf29ca2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DigitalOutput>)))
  "Returns full string definition for message of type '<DigitalOutput>"
  (cl:format cl:nil "# Digital output - only four pins. ~%~%# Array of values indices represent pins 0-3 respectively.~%bool[4] values~%~%# Set indices to true to set a pin, false to ignore.~%bool[4] mask~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DigitalOutput)))
  "Returns full string definition for message of type 'DigitalOutput"
  (cl:format cl:nil "# Digital output - only four pins. ~%~%# Array of values indices represent pins 0-3 respectively.~%bool[4] values~%~%# Set indices to true to set a pin, false to ignore.~%bool[4] mask~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DigitalOutput>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'values) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'mask) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DigitalOutput>))
  "Converts a ROS message object to a list"
  (cl:list 'DigitalOutput
    (cl:cons ':values (values msg))
    (cl:cons ':mask (mask msg))
))
