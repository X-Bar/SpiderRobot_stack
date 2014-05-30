; Auto-generated. Do not edit!


(cl:in-package SpiderRobot_pkg-msg)


;//! \htmlinclude MyChar.msg.html

(cl:defclass <MyChar> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:fixnum
    :initform 0))
)

(cl:defclass MyChar (<MyChar>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MyChar>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MyChar)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SpiderRobot_pkg-msg:<MyChar> is deprecated: use SpiderRobot_pkg-msg:MyChar instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <MyChar>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpiderRobot_pkg-msg:data-val is deprecated.  Use SpiderRobot_pkg-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MyChar>) ostream)
  "Serializes a message object of type '<MyChar>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MyChar>) istream)
  "Deserializes a message object of type '<MyChar>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MyChar>)))
  "Returns string type for a message object of type '<MyChar>"
  "SpiderRobot_pkg/MyChar")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MyChar)))
  "Returns string type for a message object of type 'MyChar"
  "SpiderRobot_pkg/MyChar")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MyChar>)))
  "Returns md5sum for a message object of type '<MyChar>"
  "8524586e34fbd7cb1c08c5f5f1ca0e57")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MyChar)))
  "Returns md5sum for a message object of type 'MyChar"
  "8524586e34fbd7cb1c08c5f5f1ca0e57")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MyChar>)))
  "Returns full string definition for message of type '<MyChar>"
  (cl:format cl:nil "int16 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MyChar)))
  "Returns full string definition for message of type 'MyChar"
  (cl:format cl:nil "int16 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MyChar>))
  (cl:+ 0
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MyChar>))
  "Converts a ROS message object to a list"
  (cl:list 'MyChar
    (cl:cons ':data (data msg))
))
