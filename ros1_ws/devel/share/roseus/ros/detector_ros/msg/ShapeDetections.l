;; Auto-generated. Do not edit!


(when (boundp 'detector_ros::ShapeDetections)
  (if (not (find-package "DETECTOR_ROS"))
    (make-package "DETECTOR_ROS"))
  (shadow 'ShapeDetections (find-package "DETECTOR_ROS")))
(unless (find-package "DETECTOR_ROS::SHAPEDETECTIONS")
  (make-package "DETECTOR_ROS::SHAPEDETECTIONS"))

(in-package "ROS")
;;//! \htmlinclude ShapeDetections.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass detector_ros::ShapeDetections
  :super ros::object
  :slots (_header _detections ))

(defmethod detector_ros::ShapeDetections
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:detections __detections) ())
    )
   (send-super :init)
   (setq _header __header)
   (setq _detections __detections)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:detections
   (&rest __detections)
   (if (keywordp (car __detections))
       (send* _detections __detections)
     (progn
       (if __detections (setq _detections (car __detections)))
       _detections)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; detector_ros/ShapeDetection[] _detections
    (apply #'+ (send-all _detections :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; detector_ros/ShapeDetection[] _detections
     (write-long (length _detections) s)
     (dolist (elem _detections)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; detector_ros/ShapeDetection[] _detections
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _detections (let (r) (dotimes (i n) (push (instance detector_ros::ShapeDetection :init) r)) r))
     (dolist (elem- _detections)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get detector_ros::ShapeDetections :md5sum-) "c04579eff31e78139542754e76efe173")
(setf (get detector_ros::ShapeDetections :datatype-) "detector_ros/ShapeDetections")
(setf (get detector_ros::ShapeDetections :definition-)
      "std_msgs/Header header
detector_ros/ShapeDetection[] detections
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: detector_ros/ShapeDetection
string shape
string color
int32 cx
int32 cy
float32 area
")



(provide :detector_ros/ShapeDetections "c04579eff31e78139542754e76efe173")


