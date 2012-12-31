FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C46_MountVehicle/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C46_MountVehicle/MountAction.h"
  "../msg_gen/cpp/include/C46_MountVehicle/MountGoal.h"
  "../msg_gen/cpp/include/C46_MountVehicle/MountActionGoal.h"
  "../msg_gen/cpp/include/C46_MountVehicle/MountResult.h"
  "../msg_gen/cpp/include/C46_MountVehicle/MountActionResult.h"
  "../msg_gen/cpp/include/C46_MountVehicle/MountFeedback.h"
  "../msg_gen/cpp/include/C46_MountVehicle/MountActionFeedback.h"
  "../msg/MountAction.msg"
  "../msg/MountGoal.msg"
  "../msg/MountActionGoal.msg"
  "../msg/MountResult.msg"
  "../msg/MountActionResult.msg"
  "../msg/MountFeedback.msg"
  "../msg/MountActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
