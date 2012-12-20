FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C51_CarOperation/msg"
  "../src/C51_CarOperation/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C51_CarOperation/C0C51_TRA.h"
  "../msg_gen/cpp/include/C51_CarOperation/C0C51_ST.h"
  "../msg_gen/cpp/include/C51_CarOperation/C0C51_CL.h"
  "../msg_gen/cpp/include/C51_CarOperation/C51C0_OPO.h"
  "../msg_gen/cpp/include/C51_CarOperation/C0C51_PAR.h"
  "../msg_gen/cpp/include/C51_CarOperation/C51C0_NOR.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
