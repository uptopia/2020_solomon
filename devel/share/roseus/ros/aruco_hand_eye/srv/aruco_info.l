;; Auto-generated. Do not edit!


(when (boundp 'aruco_hand_eye::aruco_info)
  (if (not (find-package "ARUCO_HAND_EYE"))
    (make-package "ARUCO_HAND_EYE"))
  (shadow 'aruco_info (find-package "ARUCO_HAND_EYE")))
(unless (find-package "ARUCO_HAND_EYE::ARUCO_INFO")
  (make-package "ARUCO_HAND_EYE::ARUCO_INFO"))
(unless (find-package "ARUCO_HAND_EYE::ARUCO_INFOREQUEST")
  (make-package "ARUCO_HAND_EYE::ARUCO_INFOREQUEST"))
(unless (find-package "ARUCO_HAND_EYE::ARUCO_INFORESPONSE")
  (make-package "ARUCO_HAND_EYE::ARUCO_INFORESPONSE"))

(in-package "ROS")





(defclass aruco_hand_eye::aruco_infoRequest
  :super ros::object
  :slots (_cmd _id ))

(defmethod aruco_hand_eye::aruco_infoRequest
  (:init
   (&key
    ((:cmd __cmd) "")
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _cmd (string __cmd))
   (setq _id (round __id))
   self)
  (:cmd
   (&optional __cmd)
   (if __cmd (setq _cmd __cmd)) _cmd)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; string _cmd
    4 (length _cmd)
    ;; int64 _id
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _cmd
       (write-long (length _cmd) s) (princ _cmd s)
     ;; int64 _id
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _id (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _id) (= (length (_id . bv)) 2)) ;; bignum
              (write-long (ash (elt (_id . bv) 0) 0) s)
              (write-long (ash (elt (_id . bv) 1) -1) s))
             ((and (class _id) (= (length (_id . bv)) 1)) ;; big1
              (write-long (elt (_id . bv) 0) s)
              (write-long (if (>= _id 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _id s)(write-long (if (>= _id 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _cmd
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _cmd (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int64 _id
#+(or :alpha :irix6 :x86_64)
      (setf _id (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _id (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass aruco_hand_eye::aruco_infoResponse
  :super ros::object
  :slots (_ids _tvecs _rvecs ))

(defmethod aruco_hand_eye::aruco_infoResponse
  (:init
   (&key
    ((:ids __ids) (make-array 0 :initial-element 0 :element-type :integer))
    ((:tvecs __tvecs) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:rvecs __rvecs) (make-array 0 :initial-element 0.0 :element-type :float))
    )
   (send-super :init)
   (setq _ids __ids)
   (setq _tvecs __tvecs)
   (setq _rvecs __rvecs)
   self)
  (:ids
   (&optional __ids)
   (if __ids (setq _ids __ids)) _ids)
  (:tvecs
   (&optional __tvecs)
   (if __tvecs (setq _tvecs __tvecs)) _tvecs)
  (:rvecs
   (&optional __rvecs)
   (if __rvecs (setq _rvecs __rvecs)) _rvecs)
  (:serialization-length
   ()
   (+
    ;; int64[] _ids
    (* 8    (length _ids)) 4
    ;; float64[] _tvecs
    (* 8    (length _tvecs)) 4
    ;; float64[] _rvecs
    (* 8    (length _rvecs)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64[] _ids
     (write-long (length _ids) s)
     (dotimes (i (length _ids))
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke (elt _ids i) (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class (elt _ids i)) (= (length ((elt _ids i) . bv)) 2)) ;; bignum
              (write-long (ash (elt ((elt _ids i) . bv) 0) 0) s)
              (write-long (ash (elt ((elt _ids i) . bv) 1) -1) s))
             ((and (class (elt _ids i)) (= (length ((elt _ids i) . bv)) 1)) ;; big1
              (write-long (elt ((elt _ids i) . bv) 0) s)
              (write-long (if (>= (elt _ids i) 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long (elt _ids i) s)(write-long (if (>= (elt _ids i) 0) 0 #xffffffff) s)))
       )
     ;; float64[] _tvecs
     (write-long (length _tvecs) s)
     (dotimes (i (length _tvecs))
       (sys::poke (elt _tvecs i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; float64[] _rvecs
     (write-long (length _rvecs) s)
     (dotimes (i (length _rvecs))
       (sys::poke (elt _rvecs i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64[] _ids
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _ids (instantiate integer-vector n))
     (dotimes (i n)
#+(or :alpha :irix6 :x86_64)
      (setf (elt _ids i) (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf (elt _ids i) (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
     ))
   ;; float64[] _tvecs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _tvecs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _tvecs i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; float64[] _rvecs
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _rvecs (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _rvecs i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;;
   self)
  )

(defclass aruco_hand_eye::aruco_info
  :super ros::object
  :slots ())

(setf (get aruco_hand_eye::aruco_info :md5sum-) "1bc8d5c8b1ad95abb0b2391cce4ff505")
(setf (get aruco_hand_eye::aruco_info :datatype-) "aruco_hand_eye/aruco_info")
(setf (get aruco_hand_eye::aruco_info :request) aruco_hand_eye::aruco_infoRequest)
(setf (get aruco_hand_eye::aruco_info :response) aruco_hand_eye::aruco_infoResponse)

(defmethod aruco_hand_eye::aruco_infoRequest
  (:response () (instance aruco_hand_eye::aruco_infoResponse :init)))

(setf (get aruco_hand_eye::aruco_infoRequest :md5sum-) "1bc8d5c8b1ad95abb0b2391cce4ff505")
(setf (get aruco_hand_eye::aruco_infoRequest :datatype-) "aruco_hand_eye/aruco_infoRequest")
(setf (get aruco_hand_eye::aruco_infoRequest :definition-)
      "string    cmd
int64   id

---

int64[]   ids
float64[] tvecs
float64[] rvecs

")

(setf (get aruco_hand_eye::aruco_infoResponse :md5sum-) "1bc8d5c8b1ad95abb0b2391cce4ff505")
(setf (get aruco_hand_eye::aruco_infoResponse :datatype-) "aruco_hand_eye/aruco_infoResponse")
(setf (get aruco_hand_eye::aruco_infoResponse :definition-)
      "string    cmd
int64   id

---

int64[]   ids
float64[] tvecs
float64[] rvecs

")



(provide :aruco_hand_eye/aruco_info "1bc8d5c8b1ad95abb0b2391cce4ff505")


