FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C43_LocalBodyPVA/msg"
  "../src/C43_LocalBodyPVA/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/C43_LocalBodyPVA/C43.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
