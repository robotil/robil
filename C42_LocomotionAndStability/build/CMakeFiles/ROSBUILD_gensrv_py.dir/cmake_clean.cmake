FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C42_LocomotionAndStability/msg"
  "../src/C42_LocomotionAndStability/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/C42_LocomotionAndStability/srv/__init__.py"
  "../src/C42_LocomotionAndStability/srv/_C42.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
