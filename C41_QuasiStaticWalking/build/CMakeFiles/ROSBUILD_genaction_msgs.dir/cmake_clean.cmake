FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C41_QuasiStaticWalking/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
