; Auto-generated. Do not edit!


(cl:in-package tsn_bbb_msgs-msg)


;//! \htmlinclude GPIOOut.msg.html

(cl:defclass <GPIOOut> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass GPIOOut (<GPIOOut>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GPIOOut>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GPIOOut)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name tsn_bbb_msgs-msg:<GPIOOut> is deprecated: use tsn_bbb_msgs-msg:GPIOOut instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <GPIOOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsn_bbb_msgs-msg:header-val is deprecated.  Use tsn_bbb_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GPIOOut>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader tsn_bbb_msgs-msg:data-val is deprecated.  Use tsn_bbb_msgs-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GPIOOut>) ostream)
  "Serializes a message object of type '<GPIOOut>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GPIOOut>) istream)
  "Deserializes a message object of type '<GPIOOut>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'data)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GPIOOut>)))
  "Returns string type for a message object of type '<GPIOOut>"
  "tsn_bbb_msgs/GPIOOut")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GPIOOut)))
  "Returns string type for a message object of type 'GPIOOut"
  "tsn_bbb_msgs/GPIOOut")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GPIOOut>)))
  "Returns md5sum for a message object of type '<GPIOOut>"
  "dd7703d5078812384ba611ee20b8fecf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GPIOOut)))
  "Returns md5sum for a message object of type 'GPIOOut"
  "dd7703d5078812384ba611ee20b8fecf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GPIOOut>)))
  "Returns full string definition for message of type '<GPIOOut>"
  (cl:format cl:nil "#Time Stamp of Data Point~%Header header~%~%#Current Setting of Output GPIO~%uint8 data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GPIOOut)))
  "Returns full string definition for message of type 'GPIOOut"
  (cl:format cl:nil "#Time Stamp of Data Point~%Header header~%~%#Current Setting of Output GPIO~%uint8 data~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GPIOOut>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GPIOOut>))
  "Converts a ROS message object to a list"
  (cl:list 'GPIOOut
    (cl:cons ':header (header msg))
    (cl:cons ':data (data msg))
))
