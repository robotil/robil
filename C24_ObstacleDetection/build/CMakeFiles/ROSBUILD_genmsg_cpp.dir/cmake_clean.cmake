FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C24_ObstacleDetection/msg"
  "../src/C24_ObstacleDetection/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C24_ObstacleDetection/C0C24_LAZ.h"
  "../msg_gen/cpp/include/C24_ObstacleDetection/C0C24_CAM.h"
  "../msg_gen/cpp/include/C24_ObstacleDetection/TBD.h"
  "../msg_gen/cpp/include/C24_ObstacleDetection/C24C0_ODIM.h"
  "../msg_gen/cpp/include/C24_ObstacleDetection/C0C24_SIZ.h"
  "../msg_gen/cpp/include/C24_ObstacleDetection/C24C0_OD.h"
  "../msg_gen/cpp/include/C24_ObstacleDetection/C0C24_AZI.h"
  "../msg_gen/cpp/include/C24_ObstacleDetection/C24C0_OPO.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
