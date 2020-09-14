;; Auto-generated. Do not edit!


(when (boundp 'yolo_detection::ROI)
  (if (not (find-package "YOLO_DETECTION"))
    (make-package "YOLO_DETECTION"))
  (shadow 'ROI (find-package "YOLO_DETECTION")))
(unless (find-package "YOLO_DETECTION::ROI")
  (make-package "YOLO_DETECTION::ROI"))

(in-package "ROS")
;;//! \htmlinclude ROI.msg.html


(defclass yolo_detection::ROI
  :super ros::object
  :slots (_object_name _x _y ))

(defmethod yolo_detection::ROI
  (:init
   (&key
    ((:object_name __object_name) "")
    ((:x __x) 0)
    ((:y __y) 0)
    )
   (send-super :init)
   (setq _object_name (string __object_name))
   (setq _x (round __x))
   (setq _y (round __y))
   self)
  (:object_name
   (&optional __object_name)
   (if __object_name (setq _object_name __object_name)) _object_name)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:serialization-length
   ()
   (+
    ;; string _object_name
    4 (length _object_name)
    ;; int32 _x
    4
    ;; int32 _y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _object_name
       (write-long (length _object_name) s) (princ _object_name s)
     ;; int32 _x
       (write-long _x s)
     ;; int32 _y
       (write-long _y s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _object_name
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _object_name (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _x
     (setq _x (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _y
     (setq _y (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get yolo_detection::ROI :md5sum-) "aac12b3583057ebcad7306916635123f")
(setf (get yolo_detection::ROI :datatype-) "yolo_detection/ROI")
(setf (get yolo_detection::ROI :definition-)
      "string object_name
int32 x
int32 y

")



(provide :yolo_detection/ROI "aac12b3583057ebcad7306916635123f")


