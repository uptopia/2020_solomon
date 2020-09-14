; Auto-generated. Do not edit!


(cl:in-package aruco_hand_eye-srv)


;//! \htmlinclude aruco_info-request.msg.html

(cl:defclass <aruco_info-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform "")
   (id
    :reader id
    :initarg :id
    :type cl:integer
    :initform 0))
)

(cl:defclass aruco_info-request (<aruco_info-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <aruco_info-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'aruco_info-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aruco_hand_eye-srv:<aruco_info-request> is deprecated: use aruco_hand_eye-srv:aruco_info-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <aruco_info-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:cmd-val is deprecated.  Use aruco_hand_eye-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <aruco_info-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:id-val is deprecated.  Use aruco_hand_eye-srv:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <aruco_info-request>) ostream)
  "Serializes a message object of type '<aruco_info-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <aruco_info-request>) istream)
  "Deserializes a message object of type '<aruco_info-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<aruco_info-request>)))
  "Returns string type for a service object of type '<aruco_info-request>"
  "aruco_hand_eye/aruco_infoRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aruco_info-request)))
  "Returns string type for a service object of type 'aruco_info-request"
  "aruco_hand_eye/aruco_infoRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<aruco_info-request>)))
  "Returns md5sum for a message object of type '<aruco_info-request>"
  "1bc8d5c8b1ad95abb0b2391cce4ff505")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'aruco_info-request)))
  "Returns md5sum for a message object of type 'aruco_info-request"
  "1bc8d5c8b1ad95abb0b2391cce4ff505")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<aruco_info-request>)))
  "Returns full string definition for message of type '<aruco_info-request>"
  (cl:format cl:nil "string    cmd~%int64   id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'aruco_info-request)))
  "Returns full string definition for message of type 'aruco_info-request"
  (cl:format cl:nil "string    cmd~%int64   id~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <aruco_info-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cmd))
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <aruco_info-request>))
  "Converts a ROS message object to a list"
  (cl:list 'aruco_info-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':id (id msg))
))
;//! \htmlinclude aruco_info-response.msg.html

(cl:defclass <aruco_info-response> (roslisp-msg-protocol:ros-message)
  ((ids
    :reader ids
    :initarg :ids
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0))
   (tvecs
    :reader tvecs
    :initarg :tvecs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (rvecs
    :reader rvecs
    :initarg :rvecs
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass aruco_info-response (<aruco_info-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <aruco_info-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'aruco_info-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aruco_hand_eye-srv:<aruco_info-response> is deprecated: use aruco_hand_eye-srv:aruco_info-response instead.")))

(cl:ensure-generic-function 'ids-val :lambda-list '(m))
(cl:defmethod ids-val ((m <aruco_info-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:ids-val is deprecated.  Use aruco_hand_eye-srv:ids instead.")
  (ids m))

(cl:ensure-generic-function 'tvecs-val :lambda-list '(m))
(cl:defmethod tvecs-val ((m <aruco_info-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:tvecs-val is deprecated.  Use aruco_hand_eye-srv:tvecs instead.")
  (tvecs m))

(cl:ensure-generic-function 'rvecs-val :lambda-list '(m))
(cl:defmethod rvecs-val ((m <aruco_info-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:rvecs-val is deprecated.  Use aruco_hand_eye-srv:rvecs instead.")
  (rvecs m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <aruco_info-response>) ostream)
  "Serializes a message object of type '<aruco_info-response>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ids))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    ))
   (cl:slot-value msg 'ids))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'tvecs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'tvecs))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'rvecs))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'rvecs))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <aruco_info-response>) istream)
  "Deserializes a message object of type '<aruco_info-response>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ids) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ids)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616)))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'tvecs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'tvecs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'rvecs) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'rvecs)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<aruco_info-response>)))
  "Returns string type for a service object of type '<aruco_info-response>"
  "aruco_hand_eye/aruco_infoResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aruco_info-response)))
  "Returns string type for a service object of type 'aruco_info-response"
  "aruco_hand_eye/aruco_infoResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<aruco_info-response>)))
  "Returns md5sum for a message object of type '<aruco_info-response>"
  "1bc8d5c8b1ad95abb0b2391cce4ff505")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'aruco_info-response)))
  "Returns md5sum for a message object of type 'aruco_info-response"
  "1bc8d5c8b1ad95abb0b2391cce4ff505")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<aruco_info-response>)))
  "Returns full string definition for message of type '<aruco_info-response>"
  (cl:format cl:nil "~%int64[]   ids~%float64[] tvecs~%float64[] rvecs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'aruco_info-response)))
  "Returns full string definition for message of type 'aruco_info-response"
  (cl:format cl:nil "~%int64[]   ids~%float64[] tvecs~%float64[] rvecs~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <aruco_info-response>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ids) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'tvecs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'rvecs) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <aruco_info-response>))
  "Converts a ROS message object to a list"
  (cl:list 'aruco_info-response
    (cl:cons ':ids (ids msg))
    (cl:cons ':tvecs (tvecs msg))
    (cl:cons ':rvecs (rvecs msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'aruco_info)))
  'aruco_info-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'aruco_info)))
  'aruco_info-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'aruco_info)))
  "Returns string type for a service object of type '<aruco_info>"
  "aruco_hand_eye/aruco_info")