; Auto-generated. Do not edit!


(cl:in-package color_gradient_vision-msg)


;//! \htmlinclude ColorAndPositionPairs.msg.html

(cl:defclass <ColorAndPositionPairs> (roslisp-msg-protocol:ros-message)
  ((pairs
    :reader pairs
    :initarg :pairs
    :type (cl:vector color_gradient_vision-msg:ColorAndPosition)
   :initform (cl:make-array 0 :element-type 'color_gradient_vision-msg:ColorAndPosition :initial-element (cl:make-instance 'color_gradient_vision-msg:ColorAndPosition))))
)

(cl:defclass ColorAndPositionPairs (<ColorAndPositionPairs>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ColorAndPositionPairs>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ColorAndPositionPairs)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name color_gradient_vision-msg:<ColorAndPositionPairs> is deprecated: use color_gradient_vision-msg:ColorAndPositionPairs instead.")))

(cl:ensure-generic-function 'pairs-val :lambda-list '(m))
(cl:defmethod pairs-val ((m <ColorAndPositionPairs>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader color_gradient_vision-msg:pairs-val is deprecated.  Use color_gradient_vision-msg:pairs instead.")
  (pairs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ColorAndPositionPairs>) ostream)
  "Serializes a message object of type '<ColorAndPositionPairs>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pairs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'pairs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ColorAndPositionPairs>) istream)
  "Deserializes a message object of type '<ColorAndPositionPairs>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pairs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pairs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'color_gradient_vision-msg:ColorAndPosition))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ColorAndPositionPairs>)))
  "Returns string type for a message object of type '<ColorAndPositionPairs>"
  "color_gradient_vision/ColorAndPositionPairs")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColorAndPositionPairs)))
  "Returns string type for a message object of type 'ColorAndPositionPairs"
  "color_gradient_vision/ColorAndPositionPairs")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ColorAndPositionPairs>)))
  "Returns md5sum for a message object of type '<ColorAndPositionPairs>"
  "cc8e80026229df02835845177e306a2f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ColorAndPositionPairs)))
  "Returns md5sum for a message object of type 'ColorAndPositionPairs"
  "cc8e80026229df02835845177e306a2f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ColorAndPositionPairs>)))
  "Returns full string definition for message of type '<ColorAndPositionPairs>"
  (cl:format cl:nil "ColorAndPosition[] pairs~%~%================================================================================~%MSG: color_gradient_vision/ColorAndPosition~%float32 x~%float32 y~%uint8 R~%uint8 G~%uint8 B~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ColorAndPositionPairs)))
  "Returns full string definition for message of type 'ColorAndPositionPairs"
  (cl:format cl:nil "ColorAndPosition[] pairs~%~%================================================================================~%MSG: color_gradient_vision/ColorAndPosition~%float32 x~%float32 y~%uint8 R~%uint8 G~%uint8 B~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ColorAndPositionPairs>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pairs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ColorAndPositionPairs>))
  "Converts a ROS message object to a list"
  (cl:list 'ColorAndPositionPairs
    (cl:cons ':pairs (pairs msg))
))
