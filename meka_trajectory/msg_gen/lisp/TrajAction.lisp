; Auto-generated. Do not edit!


<<<<<<< HEAD
(cl:in-package simple_traj_server-msg)
=======
(cl:in-package meka_trajectory-msg)
>>>>>>> a95bc939bd8284654725073b92d29398a091455b


;//! \htmlinclude TrajAction.msg.html

(cl:defclass <TrajAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
<<<<<<< HEAD
    :type simple_traj_server-msg:TrajActionGoal
    :initform (cl:make-instance 'simple_traj_server-msg:TrajActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type simple_traj_server-msg:TrajActionResult
    :initform (cl:make-instance 'simple_traj_server-msg:TrajActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type simple_traj_server-msg:TrajActionFeedback
    :initform (cl:make-instance 'simple_traj_server-msg:TrajActionFeedback)))
=======
    :type meka_trajectory-msg:TrajActionGoal
    :initform (cl:make-instance 'meka_trajectory-msg:TrajActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type meka_trajectory-msg:TrajActionResult
    :initform (cl:make-instance 'meka_trajectory-msg:TrajActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type meka_trajectory-msg:TrajActionFeedback
    :initform (cl:make-instance 'meka_trajectory-msg:TrajActionFeedback)))
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
)

(cl:defclass TrajAction (<TrajAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TrajAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TrajAction)
<<<<<<< HEAD
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name simple_traj_server-msg:<TrajAction> is deprecated: use simple_traj_server-msg:TrajAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <TrajAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_traj_server-msg:action_goal-val is deprecated.  Use simple_traj_server-msg:action_goal instead.")
=======
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name meka_trajectory-msg:<TrajAction> is deprecated: use meka_trajectory-msg:TrajAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <TrajAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meka_trajectory-msg:action_goal-val is deprecated.  Use meka_trajectory-msg:action_goal instead.")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <TrajAction>))
<<<<<<< HEAD
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_traj_server-msg:action_result-val is deprecated.  Use simple_traj_server-msg:action_result instead.")
=======
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meka_trajectory-msg:action_result-val is deprecated.  Use meka_trajectory-msg:action_result instead.")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <TrajAction>))
<<<<<<< HEAD
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader simple_traj_server-msg:action_feedback-val is deprecated.  Use simple_traj_server-msg:action_feedback instead.")
=======
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader meka_trajectory-msg:action_feedback-val is deprecated.  Use meka_trajectory-msg:action_feedback instead.")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TrajAction>) ostream)
  "Serializes a message object of type '<TrajAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TrajAction>) istream)
  "Deserializes a message object of type '<TrajAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TrajAction>)))
  "Returns string type for a message object of type '<TrajAction>"
<<<<<<< HEAD
  "simple_traj_server/TrajAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajAction)))
  "Returns string type for a message object of type 'TrajAction"
  "simple_traj_server/TrajAction")
=======
  "meka_trajectory/TrajAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TrajAction)))
  "Returns string type for a message object of type 'TrajAction"
  "meka_trajectory/TrajAction")
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TrajAction>)))
  "Returns md5sum for a message object of type '<TrajAction>"
  "368cde325f76ad543f39a85373230b73")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TrajAction)))
  "Returns md5sum for a message object of type 'TrajAction"
  "368cde325f76ad543f39a85373230b73")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TrajAction>)))
  "Returns full string definition for message of type '<TrajAction>"
<<<<<<< HEAD
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%TrajActionGoal action_goal~%TrajActionResult action_result~%TrajActionFeedback action_feedback~%~%================================================================================~%MSG: simple_traj_server/TrajActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%TrajGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: simple_traj_server/TrajGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%================================================================================~%MSG: simple_traj_server/TrajActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%TrajResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: simple_traj_server/TrajResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%================================================================================~%MSG: simple_traj_server/TrajActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%TrajFeedback feedback~%~%================================================================================~%MSG: simple_traj_server/TrajFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajAction)))
  "Returns full string definition for message of type 'TrajAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%TrajActionGoal action_goal~%TrajActionResult action_result~%TrajActionFeedback action_feedback~%~%================================================================================~%MSG: simple_traj_server/TrajActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%TrajGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: simple_traj_server/TrajGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%================================================================================~%MSG: simple_traj_server/TrajActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%TrajResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: simple_traj_server/TrajResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%================================================================================~%MSG: simple_traj_server/TrajActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%TrajFeedback feedback~%~%================================================================================~%MSG: simple_traj_server/TrajFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%~%"))
=======
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%TrajActionGoal action_goal~%TrajActionResult action_result~%TrajActionFeedback action_feedback~%~%================================================================================~%MSG: meka_trajectory/TrajActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%TrajGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: meka_trajectory/TrajGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%================================================================================~%MSG: meka_trajectory/TrajActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%TrajResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: meka_trajectory/TrajResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%================================================================================~%MSG: meka_trajectory/TrajActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%TrajFeedback feedback~%~%================================================================================~%MSG: meka_trajectory/TrajFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TrajAction)))
  "Returns full string definition for message of type 'TrajAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%TrajActionGoal action_goal~%TrajActionResult action_result~%TrajActionFeedback action_feedback~%~%================================================================================~%MSG: meka_trajectory/TrajActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%TrajGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: meka_trajectory/TrajGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%trajectory_msgs/JointTrajectory trajectory~%~%================================================================================~%MSG: trajectory_msgs/JointTrajectory~%Header header~%string[] joint_names~%JointTrajectoryPoint[] points~%================================================================================~%MSG: trajectory_msgs/JointTrajectoryPoint~%float64[] positions~%float64[] velocities~%float64[] accelerations~%duration time_from_start~%================================================================================~%MSG: meka_trajectory/TrajActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%TrajResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: meka_trajectory/TrajResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%================================================================================~%MSG: meka_trajectory/TrajActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%TrajFeedback feedback~%~%================================================================================~%MSG: meka_trajectory/TrajFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%~%~%"))
>>>>>>> a95bc939bd8284654725073b92d29398a091455b
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TrajAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TrajAction>))
  "Converts a ROS message object to a list"
  (cl:list 'TrajAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
