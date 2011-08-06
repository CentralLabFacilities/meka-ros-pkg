FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/simple_traj_server/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
