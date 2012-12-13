FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C45_PostureControl/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C45_PostureControl/PostureControlAction.h"
  "../msg_gen/cpp/include/C45_PostureControl/PostureControlGoal.h"
  "../msg_gen/cpp/include/C45_PostureControl/PostureControlActionGoal.h"
  "../msg_gen/cpp/include/C45_PostureControl/PostureControlResult.h"
  "../msg_gen/cpp/include/C45_PostureControl/PostureControlActionResult.h"
  "../msg_gen/cpp/include/C45_PostureControl/PostureControlFeedback.h"
  "../msg_gen/cpp/include/C45_PostureControl/PostureControlActionFeedback.h"
  "../msg/PostureControlAction.msg"
  "../msg/PostureControlGoal.msg"
  "../msg/PostureControlActionGoal.msg"
  "../msg/PostureControlResult.msg"
  "../msg/PostureControlActionResult.msg"
  "../msg/PostureControlFeedback.msg"
  "../msg/PostureControlActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
