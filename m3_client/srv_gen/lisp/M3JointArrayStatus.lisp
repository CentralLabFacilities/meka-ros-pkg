; Auto-generated. Do not edit!


(cl:in-package m3_client-srv)


;//! \htmlinclude M3JointArrayStatus-request.msg.html

(cl:defclass <M3JointArrayStatus-request> (roslisp-msg-protocol:ros-message)
  ((request
    :reader request
    :initarg :request
    :type cl:integer
    :initform 0))
)

(cl:defclass M3JointArrayStatus-request (<M3JointArrayStatus-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3JointArrayStatus-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3JointArrayStatus-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3JointArrayStatus-request> is deprecated: use m3_client-srv:M3JointArrayStatus-request instead.")))

(cl:ensure-generic-function 'request-val :lambda-list '(m))
(cl:defmethod request-val ((m <M3JointArrayStatus-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:request-val is deprecated.  Use m3_client-srv:request instead.")
  (request m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3JointArrayStatus-request>) ostream)
  "Serializes a message object of type '<M3JointArrayStatus-request>"
  (cl:let* ((signed (cl:slot-value msg 'request)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3JointArrayStatus-request>) istream)
  "Deserializes a message object of type '<M3JointArrayStatus-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'request) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3JointArrayStatus-request>)))
  "Returns string type for a service object of type '<M3JointArrayStatus-request>"
  "m3_client/M3JointArrayStatusRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3JointArrayStatus-request)))
  "Returns string type for a service object of type 'M3JointArrayStatus-request"
  "m3_client/M3JointArrayStatusRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3JointArrayStatus-request>)))
  "Returns md5sum for a message object of type '<M3JointArrayStatus-request>"
  "fd0ff7bf51cf3c5324e0ee2aef6015d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3JointArrayStatus-request)))
  "Returns md5sum for a message object of type 'M3JointArrayStatus-request"
  "fd0ff7bf51cf3c5324e0ee2aef6015d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3JointArrayStatus-request>)))
  "Returns full string definition for message of type '<M3JointArrayStatus-request>"
  (cl:format cl:nil "int32 request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3JointArrayStatus-request)))
  "Returns full string definition for message of type 'M3JointArrayStatus-request"
  (cl:format cl:nil "int32 request~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3JointArrayStatus-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3JointArrayStatus-request>))
  "Converts a ROS message object to a list"
  (cl:list 'M3JointArrayStatus-request
    (cl:cons ':request (request msg))
))
;//! \htmlinclude M3JointArrayStatus-response.msg.html

(cl:defclass <M3JointArrayStatus-response> (roslisp-msg-protocol:ros-message)
  ((base
    :reader base
    :initarg :base
    :type m3_client-msg:M3BaseStatus
    :initform (cl:make-instance 'm3_client-msg:M3BaseStatus))
   (motor_temp
    :reader motor_temp
    :initarg :motor_temp
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (amp_temp
    :reader amp_temp
    :initarg :amp_temp
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (current
    :reader current
    :initarg :current
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (torque
    :reader torque
    :initarg :torque
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (torquedot
    :reader torquedot
    :initarg :torquedot
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (theta
    :reader theta
    :initarg :theta
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (thetadot
    :reader thetadot
    :initarg :thetadot
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (thetadotdot
    :reader thetadotdot
    :initarg :thetadotdot
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (completed_spline_idx
    :reader completed_spline_idx
    :initarg :completed_spline_idx
    :type cl:integer
    :initform 0)
   (pwm_cmd
    :reader pwm_cmd
    :initarg :pwm_cmd
    :type (cl:vector cl:integer)
   :initform (cl:make-array 0 :element-type 'cl:integer :initial-element 0)))
)

(cl:defclass M3JointArrayStatus-response (<M3JointArrayStatus-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <M3JointArrayStatus-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'M3JointArrayStatus-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name m3_client-srv:<M3JointArrayStatus-response> is deprecated: use m3_client-srv:M3JointArrayStatus-response instead.")))

(cl:ensure-generic-function 'base-val :lambda-list '(m))
(cl:defmethod base-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:base-val is deprecated.  Use m3_client-srv:base instead.")
  (base m))

(cl:ensure-generic-function 'motor_temp-val :lambda-list '(m))
(cl:defmethod motor_temp-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:motor_temp-val is deprecated.  Use m3_client-srv:motor_temp instead.")
  (motor_temp m))

(cl:ensure-generic-function 'amp_temp-val :lambda-list '(m))
(cl:defmethod amp_temp-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:amp_temp-val is deprecated.  Use m3_client-srv:amp_temp instead.")
  (amp_temp m))

(cl:ensure-generic-function 'current-val :lambda-list '(m))
(cl:defmethod current-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:current-val is deprecated.  Use m3_client-srv:current instead.")
  (current m))

(cl:ensure-generic-function 'torque-val :lambda-list '(m))
(cl:defmethod torque-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:torque-val is deprecated.  Use m3_client-srv:torque instead.")
  (torque m))

(cl:ensure-generic-function 'torquedot-val :lambda-list '(m))
(cl:defmethod torquedot-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:torquedot-val is deprecated.  Use m3_client-srv:torquedot instead.")
  (torquedot m))

(cl:ensure-generic-function 'theta-val :lambda-list '(m))
(cl:defmethod theta-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:theta-val is deprecated.  Use m3_client-srv:theta instead.")
  (theta m))

(cl:ensure-generic-function 'thetadot-val :lambda-list '(m))
(cl:defmethod thetadot-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:thetadot-val is deprecated.  Use m3_client-srv:thetadot instead.")
  (thetadot m))

(cl:ensure-generic-function 'thetadotdot-val :lambda-list '(m))
(cl:defmethod thetadotdot-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:thetadotdot-val is deprecated.  Use m3_client-srv:thetadotdot instead.")
  (thetadotdot m))

(cl:ensure-generic-function 'completed_spline_idx-val :lambda-list '(m))
(cl:defmethod completed_spline_idx-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:completed_spline_idx-val is deprecated.  Use m3_client-srv:completed_spline_idx instead.")
  (completed_spline_idx m))

(cl:ensure-generic-function 'pwm_cmd-val :lambda-list '(m))
(cl:defmethod pwm_cmd-val ((m <M3JointArrayStatus-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader m3_client-srv:pwm_cmd-val is deprecated.  Use m3_client-srv:pwm_cmd instead.")
  (pwm_cmd m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <M3JointArrayStatus-response>) ostream)
  "Serializes a message object of type '<M3JointArrayStatus-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'base) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motor_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'motor_temp))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'amp_temp))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'amp_temp))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'current))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'current))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'torque))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'torque))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'torquedot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'torquedot))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'theta))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'theta))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'thetadot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'thetadot))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'thetadotdot))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'thetadotdot))
  (cl:let* ((signed (cl:slot-value msg 'completed_spline_idx)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'pwm_cmd))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    ))
   (cl:slot-value msg 'pwm_cmd))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <M3JointArrayStatus-response>) istream)
  "Deserializes a message object of type '<M3JointArrayStatus-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'base) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motor_temp) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motor_temp)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'amp_temp) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'amp_temp)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'current) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'current)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'torque) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'torque)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'torquedot) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'torquedot)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'theta) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'theta)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'thetadot) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'thetadot)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'thetadotdot) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'thetadotdot)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'completed_spline_idx) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'pwm_cmd) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'pwm_cmd)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<M3JointArrayStatus-response>)))
  "Returns string type for a service object of type '<M3JointArrayStatus-response>"
  "m3_client/M3JointArrayStatusResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3JointArrayStatus-response)))
  "Returns string type for a service object of type 'M3JointArrayStatus-response"
  "m3_client/M3JointArrayStatusResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<M3JointArrayStatus-response>)))
  "Returns md5sum for a message object of type '<M3JointArrayStatus-response>"
  "fd0ff7bf51cf3c5324e0ee2aef6015d7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'M3JointArrayStatus-response)))
  "Returns md5sum for a message object of type 'M3JointArrayStatus-response"
  "fd0ff7bf51cf3c5324e0ee2aef6015d7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<M3JointArrayStatus-response>)))
  "Returns full string definition for message of type '<M3JointArrayStatus-response>"
  (cl:format cl:nil "M3BaseStatus base~%float32[] motor_temp~%float32[] amp_temp~%float32[] current~%float32[] torque~%float32[] torquedot~%float32[] theta~%float32[] thetadot~%float32[] thetadotdot~%int32 completed_spline_idx~%int32[] pwm_cmd~%~%================================================================================~%MSG: m3_client/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'M3JointArrayStatus-response)))
  "Returns full string definition for message of type 'M3JointArrayStatus-response"
  (cl:format cl:nil "M3BaseStatus base~%float32[] motor_temp~%float32[] amp_temp~%float32[] current~%float32[] torque~%float32[] torquedot~%float32[] theta~%float32[] thetadot~%float32[] thetadotdot~%int32 completed_spline_idx~%int32[] pwm_cmd~%~%================================================================================~%MSG: m3_client/M3BaseStatus~%string name~%uint8 state~%int64 timestamp~%string rate~%string version~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <M3JointArrayStatus-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'base))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motor_temp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'amp_temp) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'current) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'torque) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'torquedot) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'theta) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'thetadot) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'thetadotdot) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'pwm_cmd) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <M3JointArrayStatus-response>))
  "Converts a ROS message object to a list"
  (cl:list 'M3JointArrayStatus-response
    (cl:cons ':base (base msg))
    (cl:cons ':motor_temp (motor_temp msg))
    (cl:cons ':amp_temp (amp_temp msg))
    (cl:cons ':current (current msg))
    (cl:cons ':torque (torque msg))
    (cl:cons ':torquedot (torquedot msg))
    (cl:cons ':theta (theta msg))
    (cl:cons ':thetadot (thetadot msg))
    (cl:cons ':thetadotdot (thetadotdot msg))
    (cl:cons ':completed_spline_idx (completed_spline_idx msg))
    (cl:cons ':pwm_cmd (pwm_cmd msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'M3JointArrayStatus)))
  'M3JointArrayStatus-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'M3JointArrayStatus)))
  'M3JointArrayStatus-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'M3JointArrayStatus)))
  "Returns string type for a service object of type '<M3JointArrayStatus>"
  "m3_client/M3JointArrayStatus")