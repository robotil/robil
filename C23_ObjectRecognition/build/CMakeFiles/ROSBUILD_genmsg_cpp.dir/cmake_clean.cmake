FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C23_ObjectRecognition/msg"
  "../src/C23_ObjectRecognition/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C23_ObjectRecognition/TBD.h"
  "../msg_gen/cpp/include/C23_ObjectRecognition/C23C0_OPO.h"
  "../msg_gen/cpp/include/C23_ObjectRecognition/C23C0_OD.h"
  "../msg_gen/cpp/include/C23_ObjectRecognition/C0C23_SEC.h"
  "../msg_gen/cpp/include/C23_ObjectRecognition/C0C23_SAR.h"
  "../msg_gen/cpp/include/C23_ObjectRecognition/C23C0_ODIM.h"
  "../msg_gen/cpp/include/C23_ObjectRecognition/C0C23_SEOB.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
