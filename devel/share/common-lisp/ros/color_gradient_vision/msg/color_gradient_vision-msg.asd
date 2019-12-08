
(cl:in-package :asdf)

(defsystem "color_gradient_vision-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ColorAndPosition" :depends-on ("_package_ColorAndPosition"))
    (:file "_package_ColorAndPosition" :depends-on ("_package"))
    (:file "ColorAndPositionPairs" :depends-on ("_package_ColorAndPositionPairs"))
    (:file "_package_ColorAndPositionPairs" :depends-on ("_package"))
  ))