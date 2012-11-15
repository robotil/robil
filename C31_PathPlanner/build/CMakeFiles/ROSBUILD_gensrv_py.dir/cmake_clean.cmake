FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C31_PathPlanner/msg"
  "../src/C31_PathPlanner/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/C31_PathPlanner/srv/__init__.py"
  "../src/C31_PathPlanner/srv/_PathPlan.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
