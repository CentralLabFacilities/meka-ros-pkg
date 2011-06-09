FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/simple_traj_server/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/simple_traj_server/msg/__init__.py"
  "../src/simple_traj_server/msg/_TrajAction.py"
  "../src/simple_traj_server/msg/_TrajGoal.py"
  "../src/simple_traj_server/msg/_TrajActionGoal.py"
  "../src/simple_traj_server/msg/_TrajResult.py"
  "../src/simple_traj_server/msg/_TrajActionResult.py"
  "../src/simple_traj_server/msg/_TrajFeedback.py"
  "../src/simple_traj_server/msg/_TrajActionFeedback.py"
  "../msg/TrajAction.msg"
  "../msg/TrajGoal.msg"
  "../msg/TrajActionGoal.msg"
  "../msg/TrajResult.msg"
  "../msg/TrajActionResult.msg"
  "../msg/TrajFeedback.msg"
  "../msg/TrajActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
