; Auto-generated. Do not edit!


(cl:in-package color_gradient_vision-msg)


;//! \htmlinclude ColorAndPosition.msg.html

(cl:defclass <ColorAndPosition> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (R
    :reader R
    :initarg :R
    :type cl:fixnum
    :initform 0)
   (G
    :reader G
    :initarg :G
    :type cl:fixnum
    :initform 0)
   (B
    :reader B
    :initarg :B
    :type cl:fixnum
    :initform 0))
)

(cl:defclass ColorAndPosition (<ColorAndPosition>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ColorAndPosition>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ColorAndPosition)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name color_gradient_vision-msg:<ColorAndPosition> is deprecated: use color_gradient_vision-msg:ColorAndPosition instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <ColorAndPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader color_gradient_vision-msg:x-val is deprecated.  Use color_gradient_vision-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <ColorAndPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader color_gradient_vision-msg:y-val is deprecated.  Use color_gradient_vision-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'R-val :lambda-list '(m))
(cl:defmethod R-val ((m <ColorAndPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader color_gradient_vision-msg:R-val is deprecated.  Use color_gradient_vision-msg:R instead.")
  (R m))

(cl:ensure-generic-function 'G-val :lambda-list '(m))
(cl:defmethod G-val ((m <ColorAndPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader color_gradient_vision-msg:G-val is deprecated.  Use color_gradient_vision-msg:G instead.")
  (G m))

(cl:ensure-generic-function 'B-val :lambda-list '(m))
(cl:defmethod B-val ((m <ColorAndPosition>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader color_gradient_vision-msg:B-val is deprecated.  Use color_gradient_vision-msg:B instead.")
  (B m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ColorAndPosition>) ostream)
  "Serializes a message object of type '<ColorAndPosition>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'R)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'G)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'B)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ColorAndPosition>) istream)
  "Deserializes a message object of type '<ColorAndPosition>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'R)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'G)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'B)) (cl:read-byte istream))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ColorAndPosition>)))
  "Returns string type for a message object of type '<ColorAndPosition>"
  "color_gradient_vision/ColorAndPosition")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ColorAndPosition)))
  "Returns string type for a message object of type 'ColorAndPosition"
  "color_gradient_vision/ColorAndPosition")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ColorAndPosition>)))
  "Returns md5sum for a message object of type '<ColorAndPosition>"
  "1e1019b42bbcb03ae038bf01961c3e51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ColorAndPosition)))
  "Returns md5sum for a message object of type 'ColorAndPosition"
  "1e1019b42bbcb03ae038bf01961c3e51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ColorAndPosition>)))
  "Returns full string definition for message of type '<ColorAndPosition>"
  (cl:format cl:nil "float32 x~%float32 y~%uint8 R~%uint8 G~%uint8 B~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ColorAndPosition)))
  "Returns full string definition for message of type 'ColorAndPosition"
  (cl:format cl:nil "float32 x~%float32 y~%uint8 R~%uint8 G~%uint8 B~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ColorAndPosition>))
  (cl:+ 0
     4
     4
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ColorAndPosition>))
  "Converts a ROS message object to a list"
  (cl:list 'ColorAndPosition
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':R (R msg))
    (cl:cons ':G (G msg))
    (cl:cons ':B (B msg))
))
