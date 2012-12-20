FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C0_RobilTask/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C0_RobilTask/msg/__init__.py"
  "../src/C0_RobilTask/msg/_RobilTaskAction.py"
  "../src/C0_RobilTask/msg/_RobilTaskGoal.py"
  "../src/C0_RobilTask/msg/_RobilTaskActionGoal.py"
  "../src/C0_RobilTask/msg/_RobilTaskResult.py"
  "../src/C0_RobilTask/msg/_RobilTaskActionResult.py"
  "../src/C0_RobilTask/msg/_RobilTaskFeedback.py"
  "../src/C0_RobilTask/msg/_RobilTaskActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
