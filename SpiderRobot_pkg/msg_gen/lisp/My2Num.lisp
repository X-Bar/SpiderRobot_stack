; Auto-generated. Do not edit!


(cl:in-package SpiderRobot_pkg-msg)


;//! \htmlinclude My2Num.msg.html

(cl:defclass <My2Num> (roslisp-msg-protocol:ros-message)
  ((cha
    :reader cha
    :initarg :cha
    :type cl:fixnum
    :initform 0)
   (pos
    :reader pos
    :initarg :pos
    :type cl:fixnum
    :initform 0))
)

(cl:defclass My2Num (<My2Num>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <My2Num>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'My2Num)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SpiderRobot_pkg-msg:<My2Num> is deprecated: use SpiderRobot_pkg-msg:My2Num instead.")))

(cl:ensure-generic-function 'cha-val :lambda-list '(m))
(cl:defmethod cha-val ((m <My2Num>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpiderRobot_pkg-msg:cha-val is deprecated.  Use SpiderRobot_pkg-msg:cha instead.")
  (cha m))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <My2Num>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader SpiderRobot_pkg-msg:pos-val is deprecated.  Use SpiderRobot_pkg-msg:pos instead.")
  (pos m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <My2Num>) ostream)
  "Serializes a message object of type '<My2Num>"
  (cl:let* ((signed (cl:slot-value msg 'cha)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'pos)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <My2Num>) istream)
  "Deserializes a message object of type '<My2Num>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cha) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pos) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<My2Num>)))
  "Returns string type for a message object of type '<My2Num>"
  "SpiderRobot_pkg/My2Num")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'My2Num)))
  "Returns string type for a message object of type 'My2Num"
  "SpiderRobot_pkg/My2Num")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<My2Num>)))
  "Returns md5sum for a message object of type '<My2Num>"
  "b11b2efb605ac36c546d47d65708d2c8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'My2Num)))
  "Returns md5sum for a message object of type 'My2Num"
  "b11b2efb605ac36c546d47d65708d2c8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<My2Num>)))
  "Returns full string definition for message of type '<My2Num>"
  (cl:format cl:nil "int8 cha~%int8 pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'My2Num)))
  "Returns full string definition for message of type 'My2Num"
  (cl:format cl:nil "int8 cha~%int8 pos~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <My2Num>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <My2Num>))
  "Converts a ROS message object to a list"
  (cl:list 'My2Num
    (cl:cons ':cha (cha msg))
    (cl:cons ':pos (pos msg))
))
