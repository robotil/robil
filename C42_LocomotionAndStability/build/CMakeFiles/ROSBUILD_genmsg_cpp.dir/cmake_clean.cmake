FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C42_LocomotionAndStability/msg"
  "../src/C42_LocomotionAndStability/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/C42_LocomotionAndStability/C42C34_CS.h"
  "../msg_gen/cpp/include/C42_LocomotionAndStability/C34C42_WM.h"
  "../msg_gen/cpp/include/C42_LocomotionAndStability/C34C42_PSU.h"
  "../msg_gen/cpp/include/C42_LocomotionAndStability/C42C34_EVE.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
