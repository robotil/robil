FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C11_Agent/msg"
  "../src/C11_Agent/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/C11C32_PATH.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_C11C32_PATH.lisp"
  "../msg_gen/lisp/C32C11_PATH.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_C32C11_PATH.lisp"
  "../msg_gen/lisp/C34C11_STT.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_C34C11_STT.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
