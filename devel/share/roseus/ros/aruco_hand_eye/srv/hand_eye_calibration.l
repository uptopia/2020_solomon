;; Auto-generated. Do not edit!


(when (boundp 'aruco_hand_eye::hand_eye_calibration)
  (if (not (find-package "ARUCO_HAND_EYE"))
    (make-package "ARUCO_HAND_EYE"))
  (shadow 'hand_eye_calibration (find-package "ARUCO_HAND_EYE")))
(unless (find-package "ARUCO_HAND_EYE::HAND_EYE_CALIBRATION")
  (make-package "ARUCO_HAND_EYE::HAND_EYE_CALIBRATION"))
(unless (find-package "ARUCO_HAND_EYE::HAND_EYE_CALIBRATIONREQUEST")
  (make-package "ARUCO_HAND_EYE::HAND_EYE_CALIBRATIONREQUEST"))
(unless (find-package "ARUCO_HAND_EYE::HAND_EYE_CALIBRATIONRESPONSE")
  (make-package "ARUCO_HAND_EYE::HAND_EYE_CALIBRATIONRESPONSE"))

(in-package "ROS")

(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))




(defclass aruco_hand_eye::hand_eye_calibrationRequest
  :super ros::object
  :slots (_cmd _end_trans ))

(defmethod aruco_hand_eye::hand_eye_calibrationRequest
  (:init
   (&key
    ((:cmd __cmd) "")
    ((:end_trans __end_trans) (instance geometry_msgs::Transform :init))
    )
   (send-super :init)
   (setq _cmd (string __cmd))
   (setq _end_trans __end_trans)
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:end_trans
   (&rest __end_trans)
   (if (keywordp (car __end_trans))
       (send* _end_trans __end_trans)
     (progn
       (if __end_trans (setq _end_trans (car __end_trans)))
       _end_trans)))
  (:serialization-length
   ()
   (+
    ;; string _cmd
    4 (length _cmd)
    ;; geometry_msgs/Transform _end_trans
    (send _end_trans :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _cmd
       (write-long (length _cmd) s) (princ _cmd s)
     ;; geometry_msgs/Transform _end_trans
       (send _end_trans :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; geometry_msgs/Transform _end_trans
     (send _end_trans :deserialize buf ptr-) (incf ptr- (send _end_trans :serialization-length))
   ;;
   self)
  )

(defclass aruco_hand_eye::hand_eye_calibrationResponse
  :super ros::object
  :slots (_is_done _end2cam_trans ))

(defmethod aruco_hand_eye::hand_eye_calibrationResponse
  (:init
   (&key
    ((:is_done __is_done) nil)
    ((:end2cam_trans __end2cam_trans) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _is_done __is_done)
   (setq _end2cam_trans __end2cam_trans)
   self)
  (:is_done
   (&optional __is_done)
   (if __is_done (setq _is_done __is_done)) _is_done)
  (:end2cam_trans
   (&optional __end2cam_trans)
   (if __end2cam_trans (setq _end2cam_trans __end2cam_trans)) _end2cam_trans)
  (:serialization-length
   ()
   (+
    ;; bool _is_done
    1
    ;; float64[] _end2cam_trans
    (* 8    (length _end2cam_trans)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _is_done
       (if _is_done (write-byte -1 s) (write-byte 0 s))
     ;; float64[] _end2cam_trans
     (write-long (length _end2cam_trans) s)
     (dotimes (i (length _end2cam_trans))
       (sys::poke (elt _end2cam_trans i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _is_done
     (setq _is_done (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64[] _end2cam_trans
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _end2cam_trans (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _end2cam_trans i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass aruco_hand_eye::hand_eye_calibration
  :super ros::object
  :slots ())

(setf (get aruco_hand_eye::hand_eye_calibration :md5sum-) "a0517a9a02aee46cf83b64864c094a8b")
(setf (get aruco_hand_eye::hand_eye_calibration :datatype-) "aruco_hand_eye/hand_eye_calibration")
(setf (get aruco_hand_eye::hand_eye_calibration :request) aruco_hand_eye::hand_eye_calibrationRequest)
(setf (get aruco_hand_eye::hand_eye_calibration :response) aruco_hand_eye::hand_eye_calibrationResponse)

(defmethod aruco_hand_eye::hand_eye_calibrationRequest
  (:response () (instance aruco_hand_eye::hand_eye_calibrationResponse :init)))

(setf (get aruco_hand_eye::hand_eye_calibrationRequest :md5sum-) "a0517a9a02aee46cf83b64864c094a8b")
(setf (get aruco_hand_eye::hand_eye_calibrationRequest :datatype-) "aruco_hand_eye/hand_eye_calibrationRequest")
(setf (get aruco_hand_eye::hand_eye_calibrationRequest :definition-)
      "string                  cmd
geometry_msgs/Transform end_trans


================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool                    is_done
float64[]             end2cam_trans


")

(setf (get aruco_hand_eye::hand_eye_calibrationResponse :md5sum-) "a0517a9a02aee46cf83b64864c094a8b")
(setf (get aruco_hand_eye::hand_eye_calibrationResponse :datatype-) "aruco_hand_eye/hand_eye_calibrationResponse")
(setf (get aruco_hand_eye::hand_eye_calibrationResponse :definition-)
      "string                  cmd
geometry_msgs/Transform end_trans


================================================================================
MSG: geometry_msgs/Transform
# This represents the transform between two coordinate frames in free space.

Vector3 translation
Quaternion rotation

================================================================================
MSG: geometry_msgs/Vector3
# This represents a vector in free space. 
# It is only meant to represent a direction. Therefore, it does not
# make sense to apply a translation to it (e.g., when applying a 
# generic rigid transformation to a Vector3, tf2 will only apply the
# rotation). If you want your data to be translatable too, use the
# geometry_msgs/Point message instead.

float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w
---
bool                    is_done
float64[]             end2cam_trans


")



(provide :aruco_hand_eye/hand_eye_calibration "a0517a9a02aee46cf83b64864c094a8b")


