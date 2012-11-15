FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C44_ClimbLadder/msg"
  "../src/C44_ClimbLadder/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/C44_ClimbLadder/C44.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
