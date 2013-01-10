FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/C45_PostureControl/msg"
  "src/C45_PostureControl/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "srv_gen/cpp/include/C45_PostureControl/com_error.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
