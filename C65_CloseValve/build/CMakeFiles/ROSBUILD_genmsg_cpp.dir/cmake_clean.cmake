FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C65_CloseValve/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C65_CloseValve/C65_CloseValveAction.h"
  "../msg_gen/cpp/include/C65_CloseValve/C65_CloseValveGoal.h"
  "../msg_gen/cpp/include/C65_CloseValve/C65_CloseValveActionGoal.h"
  "../msg_gen/cpp/include/C65_CloseValve/C65_CloseValveResult.h"
  "../msg_gen/cpp/include/C65_CloseValve/C65_CloseValveActionResult.h"
  "../msg_gen/cpp/include/C65_CloseValve/C65_CloseValveFeedback.h"
  "../msg_gen/cpp/include/C65_CloseValve/C65_CloseValveActionFeedback.h"
  "../msg/C65_CloseValveAction.msg"
  "../msg/C65_CloseValveGoal.msg"
  "../msg/C65_CloseValveActionGoal.msg"
  "../msg/C65_CloseValveResult.msg"
  "../msg/C65_CloseValveActionResult.msg"
  "../msg/C65_CloseValveFeedback.msg"
  "../msg/C65_CloseValveActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
