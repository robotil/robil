FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C41_BodyControl/msg"
  "../src/C41_BodyControl/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C41_BodyControl/C0C41_PVA.h"
  "../msg_gen/cpp/include/C41_BodyControl/C0C41_WM.h"
  "../msg_gen/cpp/include/C41_BodyControl/C41C0_APVA.h"
  "../msg_gen/cpp/include/C41_BodyControl/C0C41_TC.h"
  "../msg_gen/cpp/include/C41_BodyControl/C0C41_LOAD.h"
  "../msg_gen/cpp/include/C41_BodyControl/C41C0_AT.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
