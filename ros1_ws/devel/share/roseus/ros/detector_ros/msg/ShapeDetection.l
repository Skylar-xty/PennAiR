;; Auto-generated. Do not edit!


(when (boundp 'detector_ros::ShapeDetection)
  (if (not (find-package "DETECTOR_ROS"))
    (make-package "DETECTOR_ROS"))
  (shadow 'ShapeDetection (find-package "DETECTOR_ROS")))
(unless (find-package "DETECTOR_ROS::SHAPEDETECTION")
  (make-package "DETECTOR_ROS::SHAPEDETECTION"))

(in-package "ROS")
;;//! \htmlinclude ShapeDetection.msg.html


(defclass detector_ros::ShapeDetection
  :super ros::object
  :slots (_shape _color _cx _cy _area ))

(defmethod detector_ros::ShapeDetection
  (:init
   (&key
    ((:shape __shape) "")
    ((:color __color) "")
    ((:cx __cx) 0)
    ((:cy __cy) 0)
    ((:area __area) 0.0)
    )
   (send-super :init)
   (setq _shape (string __shape))
   (setq _color (string __color))
   (setq _cx (round __cx))
   (setq _cy (round __cy))
   (setq _area (float __area))
   self)
  (:shape
   (&optional __shape)
   (if __shape (setq _shape __shape)) _shape)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:cx
   (&optional __cx)
   (if __cx (setq _cx __cx)) _cx)
  (:cy
   (&optional __cy)
   (if __cy (setq _cy __cy)) _cy)
  (:area
   (&optional __area)
   (if __area (setq _area __area)) _area)
  (:serialization-length
   ()
   (+
    ;; string _shape
    4 (length _shape)
    ;; string _color
    4 (length _color)
    ;; int32 _cx
    4
    ;; int32 _cy
    4
    ;; float32 _area
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _shape
       (write-long (length _shape) s) (princ _shape s)
     ;; string _color
       (write-long (length _color) s) (princ _color s)
     ;; int32 _cx
       (write-long _cx s)
     ;; int32 _cy
       (write-long _cy s)
     ;; float32 _area
       (sys::poke _area (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _shape
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _shape (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _color
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _color (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _cx
     (setq _cx (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _cy
     (setq _cy (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float32 _area
     (setq _area (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get detector_ros::ShapeDetection :md5sum-) "528c5d41c8b24d2086672a7ff9129f60")
(setf (get detector_ros::ShapeDetection :datatype-) "detector_ros/ShapeDetection")
(setf (get detector_ros::ShapeDetection :definition-)
      "string shape
string color
int32 cx
int32 cy
float32 area
")



(provide :detector_ros/ShapeDetection "528c5d41c8b24d2086672a7ff9129f60")


