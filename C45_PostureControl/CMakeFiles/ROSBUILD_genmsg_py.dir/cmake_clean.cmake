FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/C45_PostureControl/msg"
  "src/C45_PostureControl/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "src/C45_PostureControl/msg/__init__.py"
  "src/C45_PostureControl/msg/_C45_PostureControlAction.py"
  "src/C45_PostureControl/msg/_C45_PostureControlGoal.py"
  "src/C45_PostureControl/msg/_C45_PostureControlActionGoal.py"
  "src/C45_PostureControl/msg/_C45_PostureControlResult.py"
  "src/C45_PostureControl/msg/_C45_PostureControlActionResult.py"
  "src/C45_PostureControl/msg/_C45_PostureControlFeedback.py"
  "src/C45_PostureControl/msg/_C45_PostureControlActionFeedback.py"
  "src/C45_PostureControl/msg/_C45C0_EVE.py"
  "src/C45_PostureControl/msg/_C34C45_PM.py"
  "src/C45_PostureControl/msg/_C34C45_PSU.py"
  "src/C45_PostureControl/msg/_C22C45_SSL.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
