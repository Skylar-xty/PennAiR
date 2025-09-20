; Auto-generated. Do not edit!


(cl:in-package detector_ros-msg)


;//! \htmlinclude ShapeDetection.msg.html

(cl:defclass <ShapeDetection> (roslisp-msg-protocol:ros-message)
  ((shape
    :reader shape
    :initarg :shape
    :type cl:string
    :initform "")
   (color
    :reader color
    :initarg :color
    :type cl:string
    :initform "")
   (cx
    :reader cx
    :initarg :cx
    :type cl:integer
    :initform 0)
   (cy
    :reader cy
    :initarg :cy
    :type cl:integer
    :initform 0)
   (area
    :reader area
    :initarg :area
    :type cl:float
    :initform 0.0))
)

(cl:defclass ShapeDetection (<ShapeDetection>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ShapeDetection>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ShapeDetection)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name detector_ros-msg:<ShapeDetection> is deprecated: use detector_ros-msg:ShapeDetection instead.")))

(cl:ensure-generic-function 'shape-val :lambda-list '(m))
(cl:defmethod shape-val ((m <ShapeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detector_ros-msg:shape-val is deprecated.  Use detector_ros-msg:shape instead.")
  (shape m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <ShapeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detector_ros-msg:color-val is deprecated.  Use detector_ros-msg:color instead.")
  (color m))

(cl:ensure-generic-function 'cx-val :lambda-list '(m))
(cl:defmethod cx-val ((m <ShapeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detector_ros-msg:cx-val is deprecated.  Use detector_ros-msg:cx instead.")
  (cx m))

(cl:ensure-generic-function 'cy-val :lambda-list '(m))
(cl:defmethod cy-val ((m <ShapeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detector_ros-msg:cy-val is deprecated.  Use detector_ros-msg:cy instead.")
  (cy m))

(cl:ensure-generic-function 'area-val :lambda-list '(m))
(cl:defmethod area-val ((m <ShapeDetection>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader detector_ros-msg:area-val is deprecated.  Use detector_ros-msg:area instead.")
  (area m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ShapeDetection>) ostream)
  "Serializes a message object of type '<ShapeDetection>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'shape))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'shape))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'color))
  (cl:let* ((signed (cl:slot-value msg 'cx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'cy)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'area))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ShapeDetection>) istream)
  "Deserializes a message object of type '<ShapeDetection>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'shape) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'shape) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'color) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cx) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cy) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'area) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ShapeDetection>)))
  "Returns string type for a message object of type '<ShapeDetection>"
  "detector_ros/ShapeDetection")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ShapeDetection)))
  "Returns string type for a message object of type 'ShapeDetection"
  "detector_ros/ShapeDetection")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ShapeDetection>)))
  "Returns md5sum for a message object of type '<ShapeDetection>"
  "528c5d41c8b24d2086672a7ff9129f60")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ShapeDetection)))
  "Returns md5sum for a message object of type 'ShapeDetection"
  "528c5d41c8b24d2086672a7ff9129f60")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ShapeDetection>)))
  "Returns full string definition for message of type '<ShapeDetection>"
  (cl:format cl:nil "string shape~%string color~%int32 cx~%int32 cy~%float32 area~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ShapeDetection)))
  "Returns full string definition for message of type 'ShapeDetection"
  (cl:format cl:nil "string shape~%string color~%int32 cx~%int32 cy~%float32 area~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ShapeDetection>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'shape))
     4 (cl:length (cl:slot-value msg 'color))
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ShapeDetection>))
  "Converts a ROS message object to a list"
  (cl:list 'ShapeDetection
    (cl:cons ':shape (shape msg))
    (cl:cons ':color (color msg))
    (cl:cons ':cx (cx msg))
    (cl:cons ':cy (cy msg))
    (cl:cons ':area (area msg))
))
