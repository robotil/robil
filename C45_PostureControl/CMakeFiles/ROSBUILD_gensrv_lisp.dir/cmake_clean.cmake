FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/C45_PostureControl/msg"
  "src/C45_PostureControl/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "srv_gen/lisp/com_error.lisp"
  "srv_gen/lisp/_package.lisp"
  "srv_gen/lisp/_package_com_error.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
