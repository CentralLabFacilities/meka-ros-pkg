; Auto-generated. Do not edit!


<<<<<<< HEAD
(cl:in-package simple_traj_server-msg)
=======
(cl:in-package meka_trajectory-msg)
>>>>>>> a95bc939bd8284654725073b92d29398a091455b


;//! \htmlinclude TrajActionGoal.msg.html

(cl:defclass <TrajActionGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_id
    :reader goal_id
    :initarg :goal_id
    :type actionlib_msgs-msg:GoalID
    :initform (cl:make-instance 'actionlib_msgs-msg:GoalID))
   (goal
    :reader goal
    :initarg :goal
<<<<<<< HEAD
    :type simple_traj_server-msg:TrajGoal
    :initform (cl:make-instance 'simple_traj_server-msg:TrajGoal)))
=======
    :type meka_trajectory-msg:TrajGoal
    :initform (cl:make-instance 'meka_trajectory-msg:TrajGoal)))
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
)

(cl:defclass TrajActionGoal (<TrajActionGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajActionGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajActionGoal)
<<<<<<< HEAD
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_traj_server-msg:<TrajActionGoal> is deprecated: use simple_traj_server-msg:TrajActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrajActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_traj_server-msg:header-val is deprecated.  Use simple_traj_server-msg:header instead.")
=======
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name meka_trajectory-msg:<TrajActionGoal> is deprecated: use meka_trajectory-msg:TrajActionGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <TrajActionGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meka_trajectory-msg:header-val is deprecated.  Use meka_trajectory-msg:header instead.")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
  (header m))

(cl:ensure-generic-function 'goal_id-val :lambda-list '(m))
(cl:defmethod goal_id-val ((m <TrajActionGoal>))
<<<<<<< HEAD
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_traj_server-msg:goal_id-val is deprecated.  Use simple_traj_server-msg:goal_id instead.")
=======
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meka_trajectory-msg:goal_id-val is deprecated.  Use meka_trajectory-msg:goal_id instead.")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
  (goal_id m))

(cl:ensure-generic-function 'goal-val :lambda-list '(m))
(cl:defmethod goal-val ((m <TrajActionGoal>))
<<<<<<< HEAD
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_traj_server-msg:goal-val is deprecated.  Use simple_traj_server-msg:goal instead.")
=======
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meka_trajectory-msg:goal-val is deprecated.  Use meka_trajectory-msg:goal instead.")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
  (goal m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajActionGoal>) ostream)
  "Serializes a message object of type '<TrajActionGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_id) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajActionGoal>) istream)
  "Deserializes a message object of type '<TrajActionGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_id) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajActionGoal>)))
  "Returns string type for a message object of type '<TrajActionGoal>"
<<<<<<< HEAD
  "simple_traj_server/TrajActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajActionGoal)))
  "Returns string type for a message object of type 'TrajActionGoal"
  "simple_traj_server/TrajActionGoal")
=======
  "meka_trajectory/TrajActionGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajActionGoal)))
  "Returns string type for a message object of type 'TrajActionGoal"
  "meka_trajectory/TrajActionGoal")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajActionGoal>)))
  "Returns md5sum for a message object of type '<TrajActionGoal>"
  "aee77e81e3afb8d91af4939d603609d8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajActionGoal)))
  "Returns md5sum for a message object of type 'TrajActionGoal"
  "aee77e81e3afb8d91af4939d603609d8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajActionGoal>)))
  "Returns full string definition for message of type '<TrajActionGoal>"
<<<<<<< HEAD
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%TrajGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: simple_traj_server/TrajGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajActionGoal)))
  "Returns full string definition for message of type 'TrajActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%TrajGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: simple_traj_server/TrajGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
=======
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%TrajGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: meka_trajectory/TrajGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajActionGoal)))
  "Returns full string definition for message of type 'TrajActionGoal"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%TrajGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: meka_trajectory/TrajGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%~%"))
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajActionGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_id))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajActionGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajActionGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_id (goal_id msg))
    (cl:cons ':goal (goal msg))
))
