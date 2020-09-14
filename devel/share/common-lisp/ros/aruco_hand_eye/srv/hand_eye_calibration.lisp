; Auto-generated. Do not edit!


(cl:in-package aruco_hand_eye-srv)


;//! \htmlinclude hand_eye_calibration-request.msg.html

(cl:defclass <hand_eye_calibration-request> (roslisp-msg-protocol:ros-message)
  ((cmd
    :reader cmd
    :initarg :cmd
    :type cl:string
    :initform "")
   (end_trans
    :reader end_trans
    :initarg :end_trans
    :type geometry_msgs-msg:Transform
    :initform (cl:make-instance 'geometry_msgs-msg:Transform)))
)

(cl:defclass hand_eye_calibration-request (<hand_eye_calibration-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_eye_calibration-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_eye_calibration-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aruco_hand_eye-srv:<hand_eye_calibration-request> is deprecated: use aruco_hand_eye-srv:hand_eye_calibration-request instead.")))

(cl:ensure-generic-function 'cmd-val :lambda-list '(m))
(cl:defmethod cmd-val ((m <hand_eye_calibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:cmd-val is deprecated.  Use aruco_hand_eye-srv:cmd instead.")
  (cmd m))

(cl:ensure-generic-function 'end_trans-val :lambda-list '(m))
(cl:defmethod end_trans-val ((m <hand_eye_calibration-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:end_trans-val is deprecated.  Use aruco_hand_eye-srv:end_trans instead.")
  (end_trans m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_eye_calibration-request>) ostream)
  "Serializes a message object of type '<hand_eye_calibration-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'cmd))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'end_trans) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_eye_calibration-request>) istream)
  "Deserializes a message object of type '<hand_eye_calibration-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'cmd) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'cmd) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'end_trans) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_eye_calibration-request>)))
  "Returns string type for a service object of type '<hand_eye_calibration-request>"
  "aruco_hand_eye/hand_eye_calibrationRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_eye_calibration-request)))
  "Returns string type for a service object of type 'hand_eye_calibration-request"
  "aruco_hand_eye/hand_eye_calibrationRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_eye_calibration-request>)))
  "Returns md5sum for a message object of type '<hand_eye_calibration-request>"
  "a0517a9a02aee46cf83b64864c094a8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_eye_calibration-request)))
  "Returns md5sum for a message object of type 'hand_eye_calibration-request"
  "a0517a9a02aee46cf83b64864c094a8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_eye_calibration-request>)))
  "Returns full string definition for message of type '<hand_eye_calibration-request>"
  (cl:format cl:nil "string                  cmd~%geometry_msgs/Transform end_trans~%~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_eye_calibration-request)))
  "Returns full string definition for message of type 'hand_eye_calibration-request"
  (cl:format cl:nil "string                  cmd~%geometry_msgs/Transform end_trans~%~%~%================================================================================~%MSG: geometry_msgs/Transform~%# This represents the transform between two coordinate frames in free space.~%~%Vector3 translation~%Quaternion rotation~%~%================================================================================~%MSG: geometry_msgs/Vector3~%# This represents a vector in free space. ~%# It is only meant to represent a direction. Therefore, it does not~%# make sense to apply a translation to it (e.g., when applying a ~%# generic rigid transformation to a Vector3, tf2 will only apply the~%# rotation). If you want your data to be translatable too, use the~%# geometry_msgs/Point message instead.~%~%float64 x~%float64 y~%float64 z~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_eye_calibration-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'cmd))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'end_trans))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_eye_calibration-request>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_eye_calibration-request
    (cl:cons ':cmd (cmd msg))
    (cl:cons ':end_trans (end_trans msg))
))
;//! \htmlinclude hand_eye_calibration-response.msg.html

(cl:defclass <hand_eye_calibration-response> (roslisp-msg-protocol:ros-message)
  ((is_done
    :reader is_done
    :initarg :is_done
    :type cl:boolean
    :initform cl:nil)
   (end2cam_trans
    :reader end2cam_trans
    :initarg :end2cam_trans
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0)))
)

(cl:defclass hand_eye_calibration-response (<hand_eye_calibration-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <hand_eye_calibration-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'hand_eye_calibration-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name aruco_hand_eye-srv:<hand_eye_calibration-response> is deprecated: use aruco_hand_eye-srv:hand_eye_calibration-response instead.")))

(cl:ensure-generic-function 'is_done-val :lambda-list '(m))
(cl:defmethod is_done-val ((m <hand_eye_calibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:is_done-val is deprecated.  Use aruco_hand_eye-srv:is_done instead.")
  (is_done m))

(cl:ensure-generic-function 'end2cam_trans-val :lambda-list '(m))
(cl:defmethod end2cam_trans-val ((m <hand_eye_calibration-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader aruco_hand_eye-srv:end2cam_trans-val is deprecated.  Use aruco_hand_eye-srv:end2cam_trans instead.")
  (end2cam_trans m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <hand_eye_calibration-response>) ostream)
  "Serializes a message object of type '<hand_eye_calibration-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'is_done) 1 0)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'end2cam_trans))))
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
   (cl:slot-value msg 'end2cam_trans))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <hand_eye_calibration-response>) istream)
  "Deserializes a message object of type '<hand_eye_calibration-response>"
    (cl:setf (cl:slot-value msg 'is_done) (cl:not (cl:zerop (cl:read-byte istream))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'end2cam_trans) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'end2cam_trans)))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<hand_eye_calibration-response>)))
  "Returns string type for a service object of type '<hand_eye_calibration-response>"
  "aruco_hand_eye/hand_eye_calibrationResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_eye_calibration-response)))
  "Returns string type for a service object of type 'hand_eye_calibration-response"
  "aruco_hand_eye/hand_eye_calibrationResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<hand_eye_calibration-response>)))
  "Returns md5sum for a message object of type '<hand_eye_calibration-response>"
  "a0517a9a02aee46cf83b64864c094a8b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'hand_eye_calibration-response)))
  "Returns md5sum for a message object of type 'hand_eye_calibration-response"
  "a0517a9a02aee46cf83b64864c094a8b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<hand_eye_calibration-response>)))
  "Returns full string definition for message of type '<hand_eye_calibration-response>"
  (cl:format cl:nil "bool                    is_done~%float64[]             end2cam_trans~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'hand_eye_calibration-response)))
  "Returns full string definition for message of type 'hand_eye_calibration-response"
  (cl:format cl:nil "bool                    is_done~%float64[]             end2cam_trans~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <hand_eye_calibration-response>))
  (cl:+ 0
     1
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'end2cam_trans) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <hand_eye_calibration-response>))
  "Converts a ROS message object to a list"
  (cl:list 'hand_eye_calibration-response
    (cl:cons ':is_done (is_done msg))
    (cl:cons ':end2cam_trans (end2cam_trans msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'hand_eye_calibration)))
  'hand_eye_calibration-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'hand_eye_calibration)))
  'hand_eye_calibration-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'hand_eye_calibration)))
  "Returns string type for a service object of type '<hand_eye_calibration>"
  "aruco_hand_eye/hand_eye_calibration")