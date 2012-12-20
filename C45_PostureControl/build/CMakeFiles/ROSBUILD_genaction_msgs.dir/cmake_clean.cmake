FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C45_PostureControl/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
