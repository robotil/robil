FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/RobilTask/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/RobilTask/msg/__init__.py"
  "../src/RobilTask/msg/_RobilTaskAction.py"
  "../src/RobilTask/msg/_RobilTaskGoal.py"
  "../src/RobilTask/msg/_RobilTaskActionGoal.py"
  "../src/RobilTask/msg/_RobilTaskResult.py"
  "../src/RobilTask/msg/_RobilTaskActionResult.py"
  "../src/RobilTask/msg/_RobilTaskFeedback.py"
  "../src/RobilTask/msg/_RobilTaskActionFeedback.py"
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
