; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude DockInfraRed.msg.html

(cl:defclass <DockInfraRed> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass DockInfraRed (<DockInfraRed>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DockInfraRed>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DockInfraRed)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<DockInfraRed> is deprecated: use kobuki_msgs-msg:DockInfraRed instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <DockInfraRed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:header-val is deprecated.  Use kobuki_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <DockInfraRed>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:data-val is deprecated.  Use kobuki_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql '<DockInfraRed>)))
    "Constants for message type '<DockInfraRed>"
  '((:NEAR_LEFT . 1)
    (:NEAR_CENTER . 2)
    (:NEAR_RIGHT . 4)
    (:FAR_LEFT . 8)
    (:FAR_CENTER . 16)
    (:FAR_RIGHT . 32))
)
(cl:defmethod roslisp-msg-protocol:symbol-codes ((msg-type (cl:eql 'DockInfraRed)))
    "Constants for message type 'DockInfraRed"
  '((:NEAR_LEFT . 1)
    (:NEAR_CENTER . 2)
    (:NEAR_RIGHT . 4)
    (:FAR_LEFT . 8)
    (:FAR_CENTER . 16)
    (:FAR_RIGHT . 32))
)
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DockInfraRed>) ostream)
  "Serializes a message object of type '<DockInfraRed>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DockInfraRed>) istream)
  "Deserializes a message object of type '<DockInfraRed>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DockInfraRed>)))
  "Returns string type for a message object of type '<DockInfraRed>"
  "kobuki_msgs/DockInfraRed")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DockInfraRed)))
  "Returns string type for a message object of type 'DockInfraRed"
  "kobuki_msgs/DockInfraRed")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DockInfraRed>)))
  "Returns md5sum for a message object of type '<DockInfraRed>"
  "4247d91ea10269fb062c53396d016a2c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DockInfraRed)))
  "Returns md5sum for a message object of type 'DockInfraRed"
  "4247d91ea10269fb062c53396d016a2c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DockInfraRed>)))
  "Returns full string definition for message of type '<DockInfraRed>"
  (cl:format cl:nil "uint8 NEAR_LEFT   =  1~%uint8 NEAR_CENTER =  2~%uint8 NEAR_RIGHT  =  4~%uint8 FAR_LEFT    =  8~%uint8 FAR_CENTER  = 16~%uint8 FAR_RIGHT   = 32~%~%Header header~%uint8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DockInfraRed)))
  "Returns full string definition for message of type 'DockInfraRed"
  (cl:format cl:nil "uint8 NEAR_LEFT   =  1~%uint8 NEAR_CENTER =  2~%uint8 NEAR_RIGHT  =  4~%uint8 FAR_LEFT    =  8~%uint8 FAR_CENTER  = 16~%uint8 FAR_RIGHT   = 32~%~%Header header~%uint8[] data~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DockInfraRed>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DockInfraRed>))
  "Converts a ROS message object to a list"
  (cl:list 'DockInfraRed
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
