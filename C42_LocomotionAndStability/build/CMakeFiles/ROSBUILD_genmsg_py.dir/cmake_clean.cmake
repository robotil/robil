FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C42_LocomotionAndStability/msg"
  "../src/C42_LocomotionAndStability/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C42_LocomotionAndStability/msg/__init__.py"
  "../src/C42_LocomotionAndStability/msg/_C42C34_CS.py"
  "../src/C42_LocomotionAndStability/msg/_C34C42_WM.py"
  "../src/C42_LocomotionAndStability/msg/_C34C42_PSU.py"
  "../src/C42_LocomotionAndStability/msg/_C42C34_EVE.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
