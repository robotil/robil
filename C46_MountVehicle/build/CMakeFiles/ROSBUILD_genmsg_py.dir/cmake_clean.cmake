FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/C46_MountVehicle/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C46_MountVehicle/msg/__init__.py"
  "../src/C46_MountVehicle/msg/_MountAction.py"
  "../src/C46_MountVehicle/msg/_MountGoal.py"
  "../src/C46_MountVehicle/msg/_MountActionGoal.py"
  "../src/C46_MountVehicle/msg/_MountResult.py"
  "../src/C46_MountVehicle/msg/_MountActionResult.py"
  "../src/C46_MountVehicle/msg/_MountFeedback.py"
  "../src/C46_MountVehicle/msg/_MountActionFeedback.py"
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
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
