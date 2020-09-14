
(cl:in-package :asdf)

(defsystem "aruco_hand_eye-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "aruco_info" :depends-on ("_package_aruco_info"))
    (:file "_package_aruco_info" :depends-on ("_package"))
    (:file "get_curr_pos" :depends-on ("_package_get_curr_pos"))
    (:file "_package_get_curr_pos" :depends-on ("_package"))
    (:file "hand_eye_calibration" :depends-on ("_package_hand_eye_calibration"))
    (:file "_package_hand_eye_calibration" :depends-on ("_package"))
  ))