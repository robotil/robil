FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C41_QuasiStaticWalking/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C41_QuasiStaticWalking/QuasiStaticWalkingAction.h"
  "../msg_gen/cpp/include/C41_QuasiStaticWalking/QuasiStaticWalkingGoal.h"
  "../msg_gen/cpp/include/C41_QuasiStaticWalking/QuasiStaticWalkingActionGoal.h"
  "../msg_gen/cpp/include/C41_QuasiStaticWalking/QuasiStaticWalkingResult.h"
  "../msg_gen/cpp/include/C41_QuasiStaticWalking/QuasiStaticWalkingActionResult.h"
  "../msg_gen/cpp/include/C41_QuasiStaticWalking/QuasiStaticWalkingFeedback.h"
  "../msg_gen/cpp/include/C41_QuasiStaticWalking/QuasiStaticWalkingActionFeedback.h"
  "../msg/QuasiStaticWalkingAction.msg"
  "../msg/QuasiStaticWalkingGoal.msg"
  "../msg/QuasiStaticWalkingActionGoal.msg"
  "../msg/QuasiStaticWalkingResult.msg"
  "../msg/QuasiStaticWalkingActionResult.msg"
  "../msg/QuasiStaticWalkingFeedback.msg"
  "../msg/QuasiStaticWalkingActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
