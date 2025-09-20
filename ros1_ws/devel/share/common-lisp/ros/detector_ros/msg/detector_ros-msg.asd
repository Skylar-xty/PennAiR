
(cl:in-package :asdf)

(defsystem "detector_ros-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "ShapeDetection" :depends-on ("_package_ShapeDetection"))
    (:file "_package_ShapeDetection" :depends-on ("_package"))
    (:file "ShapeDetections" :depends-on ("_package_ShapeDetections"))
    (:file "_package_ShapeDetections" :depends-on ("_package"))
  ))