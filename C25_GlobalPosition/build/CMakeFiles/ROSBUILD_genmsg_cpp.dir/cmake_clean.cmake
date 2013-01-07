FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/C25_GlobalPosition/msg"
  "../src/C25_GlobalPosition/srv"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C25_GlobalPosition/C25C0_OPO.h"
  "../msg_gen/cpp/include/C25_GlobalPosition/C25C0_ROP.h"
  "../msg_gen/cpp/include/C25_GlobalPosition/C0C25_AZI.h"
  "../msg_gen/cpp/include/C25_GlobalPosition/C0C25_CAM.h"
  "../msg_gen/cpp/include/C25_GlobalPosition/C0C25_LAZ.h"
  "../msg_gen/cpp/include/C25_GlobalPosition/UTM.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
