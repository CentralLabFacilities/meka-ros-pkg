; Auto-generated. Do not edit!


<<<<<<< HEAD
(cl:in-package simple_traj_server-msg)
=======
(cl:in-package meka_trajectory-msg)
>>>>>>> a95bc939bd8284654725073b92d29398a091455b


;//! \htmlinclude TrajGoal.msg.html

(cl:defclass <TrajGoal> (roslisp-msg-protocol:ros-message)
  ((trajectory
    :reader trajectory
    :initarg :trajectory
    :type trajectory_msgs-msg:JointTrajectory
    :initform (cl:make-instance 'trajectory_msgs-msg:JointTrajectory)))
)

(cl:defclass TrajGoal (<TrajGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajGoal)
<<<<<<< HEAD
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_traj_server-msg:<TrajGoal> is deprecated: use simple_traj_server-msg:TrajGoal instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <TrajGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_traj_server-msg:trajectory-val is deprecated.  Use simple_traj_server-msg:trajectory instead.")
=======
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name meka_trajectory-msg:<TrajGoal> is deprecated: use meka_trajectory-msg:TrajGoal instead.")))

(cl:ensure-generic-function 'trajectory-val :lambda-list '(m))
(cl:defmethod trajectory-val ((m <TrajGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meka_trajectory-msg:trajectory-val is deprecated.  Use meka_trajectory-msg:trajectory instead.")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
  (trajectory m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajGoal>) ostream)
  "Serializes a message object of type '<TrajGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'trajectory) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajGoal>) istream)
  "Deserializes a message object of type '<TrajGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'trajectory) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajGoal>)))
  "Returns string type for a message object of type '<TrajGoal>"
<<<<<<< HEAD
  "simple_traj_server/TrajGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajGoal)))
  "Returns string type for a message object of type 'TrajGoal"
  "simple_traj_server/TrajGoal")
=======
  "meka_trajectory/TrajGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajGoal)))
  "Returns string type for a message object of type 'TrajGoal"
  "meka_trajectory/TrajGoal")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajGoal>)))
  "Returns md5sum for a message object of type '<TrajGoal>"
  "48a668811b715b51af6b3383511ae27f")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajGoal)))
  "Returns md5sum for a message object of type 'TrajGoal"
  "48a668811b715b51af6b3383511ae27f")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajGoal>)))
  "Returns full string definition for message of type '<TrajGoal>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajGoal)))
  "Returns full string definition for message of type 'TrajGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'trajectory))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajGoal
    (cl:cons ':trajectory (trajectory msg))
))
