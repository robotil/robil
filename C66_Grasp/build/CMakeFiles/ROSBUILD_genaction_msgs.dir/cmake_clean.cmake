FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C66_Grasp/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
  "../msg/C66_GraspAction.msg"
  "../msg/C66_GraspGoal.msg"
  "../msg/C66_GraspActionGoal.msg"
  "../msg/C66_GraspResult.msg"
  "../msg/C66_GraspActionResult.msg"
  "../msg/C66_GraspFeedback.msg"
  "../msg/C66_GraspActionFeedback.msg"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
