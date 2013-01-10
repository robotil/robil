FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/C45_PostureControl/msg"
  "src/C45_PostureControl/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
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
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
