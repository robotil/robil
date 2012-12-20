FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C31_PathPlanner/msg"
  "../src/C31_PathPlanner/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C31_PathPlanner/ppRequirements.h"
  "../msg_gen/cpp/include/C31_PathPlanner/ppCharge.h"
  "../msg_gen/cpp/include/C31_PathPlanner/ppLocation.h"
  "../msg_gen/cpp/include/C31_PathPlanner/ppConstraints.h"
  "../msg_gen/cpp/include/C31_PathPlanner/ppWaypoints.h"
  "../msg_gen/cpp/include/C31_PathPlanner/ppMap.h"
  "../msg_gen/cpp/include/C31_PathPlanner/ppPosition.h"
  "../msg_gen/cpp/include/C31_PathPlanner/ppCorridor.h"
  "../msg_gen/cpp/include/C31_PathPlanner/ppRobotDimension.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
