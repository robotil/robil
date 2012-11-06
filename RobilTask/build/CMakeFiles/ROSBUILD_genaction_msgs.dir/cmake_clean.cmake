FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/RobilTask/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
