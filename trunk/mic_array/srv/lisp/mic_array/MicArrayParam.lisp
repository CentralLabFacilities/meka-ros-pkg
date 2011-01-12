; Auto-generated. Do not edit!


(in-package mic_array-srv)


;//! \htmlinclude MicArrayParam-request.msg.html

(defclass <MicArrayParam-request> (ros-message)
  ((gains
    :reader gains-val
    :initarg :gains
    :type (vector float)
   :initform (make-array 6 :element-type 'float :initial-element 0.0))
   (window_time
    :reader window_time-val
    :initarg :window_time
    :type float
    :initform 0.0)
   (threshold
    :reader threshold-val
    :initarg :threshold
    :type float
    :initform 0.0)
   (slew_rate
    :reader slew_rate-val
    :initarg :slew_rate
    :type float
    :initform 0.0))
)
(defmethod serialize ((msg <MicArrayParam-request>) ostream)
  "Serializes a message object of type '<MicArrayParam-request>"
    (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)))(slot-value msg 'gains))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'window_time))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'threshold))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
  (let ((bits (roslisp-utils:encode-single-float-bits (slot-value msg 'slew_rate))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream))
)
(defmethod deserialize ((msg <MicArrayParam-request>) istream)
  "Deserializes a message object of type '<MicArrayParam-request>"
  (setf (slot-value msg 'gains) (make-array 6))
  (let ((vals (slot-value msg 'gains)))
    (dotimes (i 6)
(let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-single-float-bits bits)))))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'window_time) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'threshold) (roslisp-utils:decode-single-float-bits bits)))
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (slot-value msg 'slew_rate) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(defmethod ros-datatype ((msg (eql '<MicArrayParam-request>)))
  "Returns string type for a service object of type '<MicArrayParam-request>"
  "mic_array/MicArrayParamRequest")
(defmethod md5sum ((type (eql '<MicArrayParam-request>)))
  "Returns md5sum for a message object of type '<MicArrayParam-request>"
  "71d2dce81906e906b1fa047324736a46")
(defmethod message-definition ((type (eql '<MicArrayParam-request>)))
  "Returns full string definition for message of type '<MicArrayParam-request>"
  (format nil "float32[6] gains~%float32 window_time~%float32 threshold~%float32 slew_rate~%~%~%"))
(defmethod serialization-length ((msg <MicArrayParam-request>))
  (+ 0
     0 (reduce #'+ (slot-value msg 'gains) :key #'(lambda (ele) (declare (ignorable ele)) (+ 4)))
     4
     4
     4
))
(defmethod ros-message-to-list ((msg <MicArrayParam-request>))
  "Converts a ROS message object to a list"
  (list '<MicArrayParam-request>
    (cons ':gains (gains-val msg))
    (cons ':window_time (window_time-val msg))
    (cons ':threshold (threshold-val msg))
    (cons ':slew_rate (slew_rate-val msg))
))
;//! \htmlinclude MicArrayParam-response.msg.html

(defclass <MicArrayParam-response> (ros-message)
  ()
)
(defmethod serialize ((msg <MicArrayParam-response>) ostream)
  "Serializes a message object of type '<MicArrayParam-response>"
)
(defmethod deserialize ((msg <MicArrayParam-response>) istream)
  "Deserializes a message object of type '<MicArrayParam-response>"
  msg
)
(defmethod ros-datatype ((msg (eql '<MicArrayParam-response>)))
  "Returns string type for a service object of type '<MicArrayParam-response>"
  "mic_array/MicArrayParamResponse")
(defmethod md5sum ((type (eql '<MicArrayParam-response>)))
  "Returns md5sum for a message object of type '<MicArrayParam-response>"
  "71d2dce81906e906b1fa047324736a46")
(defmethod message-definition ((type (eql '<MicArrayParam-response>)))
  "Returns full string definition for message of type '<MicArrayParam-response>"
  (format nil "~%"))
(defmethod serialization-length ((msg <MicArrayParam-response>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <MicArrayParam-response>))
  "Converts a ROS message object to a list"
  (list '<MicArrayParam-response>
))
(defmethod service-request-type ((msg (eql 'MicArrayParam)))
  '<MicArrayParam-request>)
(defmethod service-response-type ((msg (eql 'MicArrayParam)))
  '<MicArrayParam-response>)
(defmethod ros-datatype ((msg (eql 'MicArrayParam)))
  "Returns string type for a service object of type '<MicArrayParam>"
  "mic_array/MicArrayParam")
