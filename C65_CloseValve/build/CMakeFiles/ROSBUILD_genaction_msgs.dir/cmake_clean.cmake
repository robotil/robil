FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C65_CloseValve/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
