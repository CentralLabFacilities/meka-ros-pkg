# Describes a Hri Task
#  name and description of the task
#  group_name is the moveit group concerned by this task
#  type is either a MOTION or a WAIT for event only
#  if interaction excepted, the interaction describes the matching event
#    or reached threshold
#  max wait time is enclosed within interaction as a timeout
#  motion and/or grasp to perform (pre-planned motions trajectories only)

string name
string description
string group_name

uint8 type
uint8 UNKNOWN=0
uint8 MOTION=1
uint8 WAIT=2

bool interaction_expected
hri_msgs/HriInteraction interaction

moveit_msgs/Grasp grasp
trajectory_msgs/JointTrajectory joint_motion


