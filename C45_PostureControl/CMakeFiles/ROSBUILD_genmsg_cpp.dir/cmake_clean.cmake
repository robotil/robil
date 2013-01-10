FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/C45_PostureControl/msg"
  "src/C45_PostureControl/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/C45_PostureControl/C45_PostureControlAction.h"
  "msg_gen/cpp/include/C45_PostureControl/C45_PostureControlGoal.h"
  "msg_gen/cpp/include/C45_PostureControl/C45_PostureControlActionGoal.h"
  "msg_gen/cpp/include/C45_PostureControl/C45_PostureControlResult.h"
  "msg_gen/cpp/include/C45_PostureControl/C45_PostureControlActionResult.h"
  "msg_gen/cpp/include/C45_PostureControl/C45_PostureControlFeedback.h"
  "msg_gen/cpp/include/C45_PostureControl/C45_PostureControlActionFeedback.h"
  "msg_gen/cpp/include/C45_PostureControl/C45C0_EVE.h"
  "msg_gen/cpp/include/C45_PostureControl/C34C45_PM.h"
  "msg_gen/cpp/include/C45_PostureControl/C34C45_PSU.h"
  "msg_gen/cpp/include/C45_PostureControl/C22C45_SSL.h"
  "msg/C45_PostureControlAction.msg"
  "msg/C45_PostureControlGoal.msg"
  "msg/C45_PostureControlActionGoal.msg"
  "msg/C45_PostureControlResult.msg"
  "msg/C45_PostureControlActionResult.msg"
  "msg/C45_PostureControlFeedback.msg"
  "msg/C45_PostureControlActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
