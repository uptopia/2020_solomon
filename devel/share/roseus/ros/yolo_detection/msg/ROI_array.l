;; Auto-generated. Do not edit!


(when (boundp 'yolo_detection::ROI_array)
  (if (not (find-package "YOLO_DETECTION"))
    (make-package "YOLO_DETECTION"))
  (shadow 'ROI_array (find-package "YOLO_DETECTION")))
(unless (find-package "YOLO_DETECTION::ROI_ARRAY")
  (make-package "YOLO_DETECTION::ROI_ARRAY"))

(in-package "ROS")
;;//! \htmlinclude ROI_array.msg.html


(defclass yolo_detection::ROI_array
  :super ros::object
  :slots (_ROI_list ))

(defmethod yolo_detection::ROI_array
  (:init
   (&key
    ((:ROI_list __ROI_list) (let (r) (dotimes (i 0) (push (instance yolo_detection::ROI :init) r)) r))
    )
   (send-super :init)
   (setq _ROI_list __ROI_list)
   self)
  (:ROI_list
   (&rest __ROI_list)
   (if (keywordp (car __ROI_list))
       (send* _ROI_list __ROI_list)
     (progn
       (if __ROI_list (setq _ROI_list (car __ROI_list)))
       _ROI_list)))
  (:serialization-length
   ()
   (+
    ;; yolo_detection/ROI[] _ROI_list
    (apply #'+ (send-all _ROI_list :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; yolo_detection/ROI[] _ROI_list
     (write-long (length _ROI_list) s)
     (dolist (elem _ROI_list)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; yolo_detection/ROI[] _ROI_list
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ROI_list (let (r) (dotimes (i n) (push (instance yolo_detection::ROI :init) r)) r))
     (dolist (elem- _ROI_list)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get yolo_detection::ROI_array :md5sum-) "98a75627170c780818a3f3f2d48f82c4")
(setf (get yolo_detection::ROI_array :datatype-) "yolo_detection/ROI_array")
(setf (get yolo_detection::ROI_array :definition-)
      "ROI[] ROI_list

================================================================================
MSG: yolo_detection/ROI
string object_name
int32 x
int32 y

")



(provide :yolo_detection/ROI_array "98a75627170c780818a3f3f2d48f82c4")


