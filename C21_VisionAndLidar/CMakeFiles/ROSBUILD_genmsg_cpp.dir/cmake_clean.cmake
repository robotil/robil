FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/c21_Vision_and_Lidar/msg"
  "src/c21_Vision_and_Lidar/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "msg_gen/cpp/include/c21_Vision_and_Lidar/C0C21_CAM.h"
  "msg_gen/cpp/include/c21_Vision_and_Lidar/C0C21_AZI.h"
  "msg_gen/cpp/include/c21_Vision_and_Lidar/C0C21_RES.h"
  "msg_gen/cpp/include/c21_Vision_and_Lidar/C21C0_3DR.h"
  "msg_gen/cpp/include/c21_Vision_and_Lidar/C0C21_SIZ.h"
  "msg_gen/cpp/include/c21_Vision_and_Lidar/C21C0_3DF.h"
  "msg_gen/cpp/include/c21_Vision_and_Lidar/C0C21_LAZ.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
