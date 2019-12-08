;; Auto-generated. Do not edit!


(when (boundp 'color_gradient_vision::ColorAndPositionPairs)
  (if (not (find-package "COLOR_GRADIENT_VISION"))
    (make-package "COLOR_GRADIENT_VISION"))
  (shadow 'ColorAndPositionPairs (find-package "COLOR_GRADIENT_VISION")))
(unless (find-package "COLOR_GRADIENT_VISION::COLORANDPOSITIONPAIRS")
  (make-package "COLOR_GRADIENT_VISION::COLORANDPOSITIONPAIRS"))

(in-package "ROS")
;;//! \htmlinclude ColorAndPositionPairs.msg.html


(defclass color_gradient_vision::ColorAndPositionPairs
  :super ros::object
  :slots (_pairs ))

(defmethod color_gradient_vision::ColorAndPositionPairs
  (:init
   (&key
    ((:pairs __pairs) (let (r) (dotimes (i 0) (push (instance color_gradient_vision::ColorAndPosition :init) r)) r))
    )
   (send-super :init)
   (setq _pairs __pairs)
   self)
  (:pairs
   (&rest __pairs)
   (if (keywordp (car __pairs))
       (send* _pairs __pairs)
     (progn
       (if __pairs (setq _pairs (car __pairs)))
       _pairs)))
  (:serialization-length
   ()
   (+
    ;; color_gradient_vision/ColorAndPosition[] _pairs
    (apply #'+ (send-all _pairs :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; color_gradient_vision/ColorAndPosition[] _pairs
     (write-long (length _pairs) s)
     (dolist (elem _pairs)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; color_gradient_vision/ColorAndPosition[] _pairs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _pairs (let (r) (dotimes (i n) (push (instance color_gradient_vision::ColorAndPosition :init) r)) r))
     (dolist (elem- _pairs)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get color_gradient_vision::ColorAndPositionPairs :md5sum-) "cc8e80026229df02835845177e306a2f")
(setf (get color_gradient_vision::ColorAndPositionPairs :datatype-) "color_gradient_vision/ColorAndPositionPairs")
(setf (get color_gradient_vision::ColorAndPositionPairs :definition-)
      "ColorAndPosition[] pairs

================================================================================
MSG: color_gradient_vision/ColorAndPosition
float32 x
float32 y
uint8 R
uint8 G
uint8 B

")



(provide :color_gradient_vision/ColorAndPositionPairs "cc8e80026229df02835845177e306a2f")


