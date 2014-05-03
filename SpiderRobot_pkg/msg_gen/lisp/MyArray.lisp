; Auto-generated. Do not edit!


(cl:in-package SpiderRobot_pkg-msg)


;//! \htmlinclude MyArray.msg.html

(cl:defclass <MyArray> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 18 :element-type 'cl:fixnum :initial-element 0))
   (size
    :reader size
    :initarg :size
    :type cl:fixnum
    :initform 0)
   (command
    :reader command
    :initarg :command
    :type cl:fixnum
    :initform 0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MyArray (<MyArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SpiderRobot_pkg-msg:<MyArray> is deprecated: use SpiderRobot_pkg-msg:MyArray instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <MyArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpiderRobot_pkg-msg:data-val is deprecated.  Use SpiderRobot_pkg-msg:data instead.")
  (data m))

(cl:ensure-generic-function 'size-val :lambda-list '(m))
(cl:defmethod size-val ((m <MyArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpiderRobot_pkg-msg:size-val is deprecated.  Use SpiderRobot_pkg-msg:size instead.")
  (size m))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <MyArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpiderRobot_pkg-msg:command-val is deprecated.  Use SpiderRobot_pkg-msg:command instead.")
  (command m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <MyArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpiderRobot_pkg-msg:speed-val is deprecated.  Use SpiderRobot_pkg-msg:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyArray>) ostream)
  "Serializes a message object of type '<MyArray>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
  (cl:let* ((signed (cl:slot-value msg 'size)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyArray>) istream)
  "Deserializes a message object of type '<MyArray>"
  (cl:setf (cl:slot-value msg 'data) (cl:make-array 18))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i 18)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'size) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyArray>)))
  "Returns string type for a message object of type '<MyArray>"
  "SpiderRobot_pkg/MyArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyArray)))
  "Returns string type for a message object of type 'MyArray"
  "SpiderRobot_pkg/MyArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyArray>)))
  "Returns md5sum for a message object of type '<MyArray>"
  "a76a9cc52d3754ba6d87d09948ffaf98")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyArray)))
  "Returns md5sum for a message object of type 'MyArray"
  "a76a9cc52d3754ba6d87d09948ffaf98")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyArray>)))
  "Returns full string definition for message of type '<MyArray>"
  (cl:format cl:nil "int16[18] data~%int8 size~%int8 command~%int16 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyArray)))
  "Returns full string definition for message of type 'MyArray"
  (cl:format cl:nil "int16[18] data~%int8 size~%int8 command~%int16 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyArray>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 2)))
     1
     1
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyArray>))
  "Converts a ROS message object to a list"
  (cl:list 'MyArray
    (cl:cons ':data (data msg))
    (cl:cons ':size (size msg))
    (cl:cons ':command (command msg))
    (cl:cons ':speed (speed msg))
))
