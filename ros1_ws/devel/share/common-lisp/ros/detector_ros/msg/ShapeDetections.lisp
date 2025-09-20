; Auto-generated. Do not edit!


(cl:in-package detector_ros-msg)


;//! \htmlinclude ShapeDetections.msg.html

(cl:defclass <ShapeDetections> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (detections
    :reader detections
    :initarg :detections
    :type (cl:vector detector_ros-msg:ShapeDetection)
   :initform (cl:make-array 0 :element-type 'detector_ros-msg:ShapeDetection :initial-element (cl:make-instance 'detector_ros-msg:ShapeDetection))))
)

(cl:defclass ShapeDetections (<ShapeDetections>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ShapeDetections>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ShapeDetections)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detector_ros-msg:<ShapeDetections> is deprecated: use detector_ros-msg:ShapeDetections instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ShapeDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detector_ros-msg:header-val is deprecated.  Use detector_ros-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'detections-val :lambda-list '(m))
(cl:defmethod detections-val ((m <ShapeDetections>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detector_ros-msg:detections-val is deprecated.  Use detector_ros-msg:detections instead.")
  (detections m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ShapeDetections>) ostream)
  "Serializes a message object of type '<ShapeDetections>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'detections))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'detections))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ShapeDetections>) istream)
  "Deserializes a message object of type '<ShapeDetections>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'detections) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'detections)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'detector_ros-msg:ShapeDetection))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ShapeDetections>)))
  "Returns string type for a message object of type '<ShapeDetections>"
  "detector_ros/ShapeDetections")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShapeDetections)))
  "Returns string type for a message object of type 'ShapeDetections"
  "detector_ros/ShapeDetections")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ShapeDetections>)))
  "Returns md5sum for a message object of type '<ShapeDetections>"
  "c04579eff31e78139542754e76efe173")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ShapeDetections)))
  "Returns md5sum for a message object of type 'ShapeDetections"
  "c04579eff31e78139542754e76efe173")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ShapeDetections>)))
  "Returns full string definition for message of type '<ShapeDetections>"
  (cl:format cl:nil "std_msgs/Header header~%detector_ros/ShapeDetection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: detector_ros/ShapeDetection~%string shape~%string color~%int32 cx~%int32 cy~%float32 area~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ShapeDetections)))
  "Returns full string definition for message of type 'ShapeDetections"
  (cl:format cl:nil "std_msgs/Header header~%detector_ros/ShapeDetection[] detections~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: detector_ros/ShapeDetection~%string shape~%string color~%int32 cx~%int32 cy~%float32 area~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ShapeDetections>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'detections) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ShapeDetections>))
  "Converts a ROS message object to a list"
  (cl:list 'ShapeDetections
    (cl:cons ':header (header msg))
    (cl:cons ':detections (detections msg))
))
