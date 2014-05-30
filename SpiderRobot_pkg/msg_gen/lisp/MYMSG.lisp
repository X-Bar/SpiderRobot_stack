; Auto-generated. Do not edit!


(cl:in-package SpiderRobot_pkg-msg)


;//! \htmlinclude MYMSG.msg.html

(cl:defclass <MYMSG> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass MYMSG (<MYMSG>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MYMSG>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MYMSG)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name SpiderRobot_pkg-msg:<MYMSG> is deprecated: use SpiderRobot_pkg-msg:MYMSG instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MYMSG>) ostream)
  "Serializes a message object of type '<MYMSG>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MYMSG>) istream)
  "Deserializes a message object of type '<MYMSG>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MYMSG>)))
  "Returns string type for a message object of type '<MYMSG>"
  "SpiderRobot_pkg/MYMSG")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MYMSG)))
  "Returns string type for a message object of type 'MYMSG"
  "SpiderRobot_pkg/MYMSG")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MYMSG>)))
  "Returns md5sum for a message object of type '<MYMSG>"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MYMSG)))
  "Returns md5sum for a message object of type 'MYMSG"
  "d41d8cd98f00b204e9800998ecf8427e")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MYMSG>)))
  "Returns full string definition for message of type '<MYMSG>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MYMSG)))
  "Returns full string definition for message of type 'MYMSG"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MYMSG>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MYMSG>))
  "Converts a ROS message object to a list"
  (cl:list 'MYMSG
))
