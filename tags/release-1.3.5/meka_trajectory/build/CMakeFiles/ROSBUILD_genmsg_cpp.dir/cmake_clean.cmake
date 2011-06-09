FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/simple_traj_server/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/simple_traj_server/TrajAction.h"
  "../msg_gen/cpp/include/simple_traj_server/TrajGoal.h"
  "../msg_gen/cpp/include/simple_traj_server/TrajActionGoal.h"
  "../msg_gen/cpp/include/simple_traj_server/TrajResult.h"
  "../msg_gen/cpp/include/simple_traj_server/TrajActionResult.h"
  "../msg_gen/cpp/include/simple_traj_server/TrajFeedback.h"
  "../msg_gen/cpp/include/simple_traj_server/TrajActionFeedback.h"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
