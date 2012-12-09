FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C66_Grasp/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C66_Grasp/msg/__init__.py"
  "../src/C66_Grasp/msg/_C66_GraspAction.py"
  "../src/C66_Grasp/msg/_C66_GraspGoal.py"
  "../src/C66_Grasp/msg/_C66_GraspActionGoal.py"
  "../src/C66_Grasp/msg/_C66_GraspResult.py"
  "../src/C66_Grasp/msg/_C66_GraspActionResult.py"
  "../src/C66_Grasp/msg/_C66_GraspFeedback.py"
  "../src/C66_Grasp/msg/_C66_GraspActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
