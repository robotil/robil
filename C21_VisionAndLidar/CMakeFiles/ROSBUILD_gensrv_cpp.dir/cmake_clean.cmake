FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/c21_Vision_and_Lidar/msg"
  "src/c21_Vision_and_Lidar/srv"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/c21_Vision_and_Lidar/C21.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
