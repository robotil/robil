FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C24_ObstacleDetection/msg"
  "../src/C24_ObstacleDetection/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/C24.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_C24.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
