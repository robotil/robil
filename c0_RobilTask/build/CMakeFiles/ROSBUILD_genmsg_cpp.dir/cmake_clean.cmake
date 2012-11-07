FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/c0_RobilTask/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/c0_RobilTask/RobilTaskAction.h"
  "../msg_gen/cpp/include/c0_RobilTask/RobilTaskGoal.h"
  "../msg_gen/cpp/include/c0_RobilTask/RobilTaskActionGoal.h"
  "../msg_gen/cpp/include/c0_RobilTask/RobilTaskResult.h"
  "../msg_gen/cpp/include/c0_RobilTask/RobilTaskActionResult.h"
  "../msg_gen/cpp/include/c0_RobilTask/RobilTaskFeedback.h"
  "../msg_gen/cpp/include/c0_RobilTask/RobilTaskActionFeedback.h"
  "../msg/RobilTaskAction.msg"
  "../msg/RobilTaskGoal.msg"
  "../msg/RobilTaskActionGoal.msg"
  "../msg/RobilTaskResult.msg"
  "../msg/RobilTaskActionResult.msg"
  "../msg/RobilTaskFeedback.msg"
  "../msg/RobilTaskActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
