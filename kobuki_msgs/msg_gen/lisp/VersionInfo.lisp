; Auto-generated. Do not edit!


(cl:in-package kobuki_msgs-msg)


;//! \htmlinclude VersionInfo.msg.html

(cl:defclass <VersionInfo> (roslisp-msg-protocol:ros-message)
  ((firmware
    :reader firmware
    :initarg :firmware
    :type cl:fixnum
    :initform 0)
   (hardware
    :reader hardware
    :initarg :hardware
    :type cl:fixnum
    :initform 0)
   (software
    :reader software
    :initarg :software
    :type cl:integer
    :initform 0)
   (udid
    :reader udid
    :initarg :udid
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass VersionInfo (<VersionInfo>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VersionInfo>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VersionInfo)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name kobuki_msgs-msg:<VersionInfo> is deprecated: use kobuki_msgs-msg:VersionInfo instead.")))

(cl:ensure-generic-function 'firmware-val :lambda-list '(m))
(cl:defmethod firmware-val ((m <VersionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:firmware-val is deprecated.  Use kobuki_msgs-msg:firmware instead.")
  (firmware m))

(cl:ensure-generic-function 'hardware-val :lambda-list '(m))
(cl:defmethod hardware-val ((m <VersionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:hardware-val is deprecated.  Use kobuki_msgs-msg:hardware instead.")
  (hardware m))

(cl:ensure-generic-function 'software-val :lambda-list '(m))
(cl:defmethod software-val ((m <VersionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:software-val is deprecated.  Use kobuki_msgs-msg:software instead.")
  (software m))

(cl:ensure-generic-function 'udid-val :lambda-list '(m))
(cl:defmethod udid-val ((m <VersionInfo>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader kobuki_msgs-msg:udid-val is deprecated.  Use kobuki_msgs-msg:udid instead.")
  (udid m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VersionInfo>) ostream)
  "Serializes a message object of type '<VersionInfo>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firmware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'firmware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hardware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hardware)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'software)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'software)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'software)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'udid))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) ele) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) ele) ostream))
   (cl:slot-value msg 'udid))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VersionInfo>) istream)
  "Deserializes a message object of type '<VersionInfo>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'firmware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'firmware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'hardware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'hardware)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'software)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'software)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'software)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'software)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'udid) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'udid)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:aref vals i)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:aref vals i)) (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VersionInfo>)))
  "Returns string type for a message object of type '<VersionInfo>"
  "kobuki_msgs/VersionInfo")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VersionInfo)))
  "Returns string type for a message object of type 'VersionInfo"
  "kobuki_msgs/VersionInfo")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VersionInfo>)))
  "Returns md5sum for a message object of type '<VersionInfo>"
  "adb07428ccdeb5dcaa56f5ef3160a76c")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VersionInfo)))
  "Returns md5sum for a message object of type 'VersionInfo"
  "adb07428ccdeb5dcaa56f5ef3160a76c")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VersionInfo>)))
  "Returns full string definition for message of type '<VersionInfo>"
  (cl:format cl:nil "# Contains version info for the kobuki platform.~%# Useful for introspection~%~%uint16 firmware~%uint16 hardware~%uint32 software~%uint32[] udid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VersionInfo)))
  "Returns full string definition for message of type 'VersionInfo"
  (cl:format cl:nil "# Contains version info for the kobuki platform.~%# Useful for introspection~%~%uint16 firmware~%uint16 hardware~%uint32 software~%uint32[] udid~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VersionInfo>))
  (cl:+ 0
     2
     2
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'udid) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VersionInfo>))
  "Converts a ROS message object to a list"
  (cl:list 'VersionInfo
    (cl:cons ':firmware (firmware msg))
    (cl:cons ':hardware (hardware msg))
    (cl:cons ':software (software msg))
    (cl:cons ':udid (udid msg))
))
