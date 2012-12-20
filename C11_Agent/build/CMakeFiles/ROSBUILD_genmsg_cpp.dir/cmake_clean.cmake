FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C11_Agent/msg"
  "../src/C11_Agent/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C11_Agent/C34C11_STT.h"
  "../msg_gen/cpp/include/C11_Agent/C23C11_OSM.h"
  "../msg_gen/cpp/include/C11_Agent/C11C23_OBP.h"
  "../msg_gen/cpp/include/C11_Agent/C32C11_PATH.h"
  "../msg_gen/cpp/include/C11_Agent/C11C23_OBM.h"
  "../msg_gen/cpp/include/C11_Agent/C11C24_OSM.h"
  "../msg_gen/cpp/include/C11_Agent/D3SPACE.h"
  "../msg_gen/cpp/include/C11_Agent/C11C32_PATH.h"
  "../msg_gen/cpp/include/C11_Agent/coord.h"
  "../msg_gen/cpp/include/C11_Agent/C24C11_OSM.h"
  "../msg_gen/cpp/include/C11_Agent/C11C24_OSP.h"
  "../msg_gen/cpp/include/C11_Agent/pathLocation.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
