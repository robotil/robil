FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C22_GroundRecognitionAndMapping/msg"
  "../src/C22_GroundRecognitionAndMapping/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C22_GroundRecognitionAndMapping/C0C22_CAM.h"
  "../msg_gen/cpp/include/C22_GroundRecognitionAndMapping/C0C22_SAF.h"
  "../msg_gen/cpp/include/C22_GroundRecognitionAndMapping/C0C22_AZI.h"
  "../msg_gen/cpp/include/C22_GroundRecognitionAndMapping/C22C0_PATH.h"
  "../msg_gen/cpp/include/C22_GroundRecognitionAndMapping/C0C22_LAZ.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
