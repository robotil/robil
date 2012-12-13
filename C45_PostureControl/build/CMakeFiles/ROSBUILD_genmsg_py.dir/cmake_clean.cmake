FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C45_PostureControl/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C45_PostureControl/msg/__init__.py"
  "../src/C45_PostureControl/msg/_PostureControlAction.py"
  "../src/C45_PostureControl/msg/_PostureControlGoal.py"
  "../src/C45_PostureControl/msg/_PostureControlActionGoal.py"
  "../src/C45_PostureControl/msg/_PostureControlResult.py"
  "../src/C45_PostureControl/msg/_PostureControlActionResult.py"
  "../src/C45_PostureControl/msg/_PostureControlFeedback.py"
  "../src/C45_PostureControl/msg/_PostureControlActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
