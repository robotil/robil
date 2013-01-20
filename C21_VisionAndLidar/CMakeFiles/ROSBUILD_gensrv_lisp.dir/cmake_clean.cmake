FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/c21_Vision_and_Lidar/msg"
  "src/c21_Vision_and_Lidar/srv"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "srv_gen/lisp/C21.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_C21.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
