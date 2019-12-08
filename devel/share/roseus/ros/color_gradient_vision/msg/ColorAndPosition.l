;; Auto-generated. Do not edit!


(when (boundp 'color_gradient_vision::ColorAndPosition)
  (if (not (find-package "COLOR_GRADIENT_VISION"))
    (make-package "COLOR_GRADIENT_VISION"))
  (shadow 'ColorAndPosition (find-package "COLOR_GRADIENT_VISION")))
(unless (find-package "COLOR_GRADIENT_VISION::COLORANDPOSITION")
  (make-package "COLOR_GRADIENT_VISION::COLORANDPOSITION"))

(in-package "ROS")
;;//! \htmlinclude ColorAndPosition.msg.html


(defclass color_gradient_vision::ColorAndPosition
  :super ros::object
  :slots (_x _y _R _G _B ))

(defmethod color_gradient_vision::ColorAndPosition
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:R __R) 0)
    ((:G __G) 0)
    ((:B __B) 0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _R (round __R))
   (setq _G (round __G))
   (setq _B (round __B))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:R
   (&optional __R)
   (if __R (setq _R __R)) _R)
  (:G
   (&optional __G)
   (if __G (setq _G __G)) _G)
  (:B
   (&optional __B)
   (if __B (setq _B __B)) _B)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; uint8 _R
    1
    ;; uint8 _G
    1
    ;; uint8 _B
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; uint8 _R
       (write-byte _R s)
     ;; uint8 _G
       (write-byte _G s)
     ;; uint8 _B
       (write-byte _B s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; uint8 _R
     (setq _R (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _G
     (setq _G (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; uint8 _B
     (setq _B (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;;
   self)
  )

(setf (get color_gradient_vision::ColorAndPosition :md5sum-) "1e1019b42bbcb03ae038bf01961c3e51")
(setf (get color_gradient_vision::ColorAndPosition :datatype-) "color_gradient_vision/ColorAndPosition")
(setf (get color_gradient_vision::ColorAndPosition :definition-)
      "float32 x
float32 y
uint8 R
uint8 G
uint8 B

")



(provide :color_gradient_vision/ColorAndPosition "1e1019b42bbcb03ae038bf01961c3e51")


