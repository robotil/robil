FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C43_LocalBodyPVA/msg"
  "../src/C43_LocalBodyPVA/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C43_LocalBodyPVA/C43C0_JPVA.h"
  "../msg_gen/cpp/include/C43_LocalBodyPVA/C0C43_SJ.h"
  "../msg_gen/cpp/include/C43_LocalBodyPVA/C0C43_SL.h"
  "../msg_gen/cpp/include/C43_LocalBodyPVA/C43C0_LPVA.h"
  "../msg_gen/cpp/include/C43_LocalBodyPVA/C0C43_SI.h"
  "../msg_gen/cpp/include/C43_LocalBodyPVA/C0C43_SLI.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
