FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C42_LocomotionAndStability/msg"
  "../src/C42_LocomotionAndStability/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/C42C34_CS.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_C42C34_CS.lisp"
  "../msg_gen/lisp/C34C42_WM.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_C34C42_WM.lisp"
  "../msg_gen/lisp/C34C42_PSU.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_C34C42_PSU.lisp"
  "../msg_gen/lisp/C42C34_EVE.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_C42C34_EVE.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
