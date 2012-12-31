FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C46_MountVehicle/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/MountAction.msg"
  "../msg/MountGoal.msg"
  "../msg/MountActionGoal.msg"
  "../msg/MountResult.msg"
  "../msg/MountActionResult.msg"
  "../msg/MountFeedback.msg"
  "../msg/MountActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
