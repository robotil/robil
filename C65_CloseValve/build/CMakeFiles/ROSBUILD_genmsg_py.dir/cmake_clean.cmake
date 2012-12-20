FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C65_CloseValve/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C65_CloseValve/msg/__init__.py"
  "../src/C65_CloseValve/msg/_C65_CloseValveAction.py"
  "../src/C65_CloseValve/msg/_C65_CloseValveGoal.py"
  "../src/C65_CloseValve/msg/_C65_CloseValveActionGoal.py"
  "../src/C65_CloseValve/msg/_C65_CloseValveResult.py"
  "../src/C65_CloseValve/msg/_C65_CloseValveActionResult.py"
  "../src/C65_CloseValve/msg/_C65_CloseValveFeedback.py"
  "../src/C65_CloseValve/msg/_C65_CloseValveActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
