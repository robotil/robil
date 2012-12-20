FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C41_QuasiStaticWalking/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C41_QuasiStaticWalking/msg/__init__.py"
  "../src/C41_QuasiStaticWalking/msg/_QuasiStaticWalkingAction.py"
  "../src/C41_QuasiStaticWalking/msg/_QuasiStaticWalkingGoal.py"
  "../src/C41_QuasiStaticWalking/msg/_QuasiStaticWalkingActionGoal.py"
  "../src/C41_QuasiStaticWalking/msg/_QuasiStaticWalkingResult.py"
  "../src/C41_QuasiStaticWalking/msg/_QuasiStaticWalkingActionResult.py"
  "../src/C41_QuasiStaticWalking/msg/_QuasiStaticWalkingFeedback.py"
  "../src/C41_QuasiStaticWalking/msg/_QuasiStaticWalkingActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
