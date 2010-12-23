FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/m3_client/M3ComponentStatus.h"
  "../srv_gen/cpp/include/m3_client/M3HumanoidCmd.h"
  "../srv_gen/cpp/include/m3_client/M3LoadX6Cmd.h"
  "../srv_gen/cpp/include/m3_client/M3LoadX6Status.h"
  "../srv_gen/cpp/include/m3_client/M3LoadX6Param.h"
  "../srv_gen/cpp/include/m3_client/M3JointArrayParam.h"
  "../srv_gen/cpp/include/m3_client/M3JointArrayCmd.h"
  "../srv_gen/cpp/include/m3_client/M3ComponentParam.h"
  "../srv_gen/cpp/include/m3_client/M3JointArrayStatus.h"
  "../srv_gen/cpp/include/m3_client/M3ComponentCmd.h"
  "../srv_gen/cpp/include/m3_client/M3HumanoidParam.h"
  "../srv_gen/cpp/include/m3_client/M3HumanoidStatus.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
