FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C31_PathPlanner/msg"
  "../src/C31_PathPlanner/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C31_PathPlanner/msg/__init__.py"
  "../src/C31_PathPlanner/msg/_ppRequirements.py"
  "../src/C31_PathPlanner/msg/_ppCharge.py"
  "../src/C31_PathPlanner/msg/_ppLocation.py"
  "../src/C31_PathPlanner/msg/_ppConstraints.py"
  "../src/C31_PathPlanner/msg/_ppWaypoints.py"
  "../src/C31_PathPlanner/msg/_ppMap.py"
  "../src/C31_PathPlanner/msg/_ppPosition.py"
  "../src/C31_PathPlanner/msg/_ppCorridor.py"
  "../src/C31_PathPlanner/msg/_ppRobotDimension.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
