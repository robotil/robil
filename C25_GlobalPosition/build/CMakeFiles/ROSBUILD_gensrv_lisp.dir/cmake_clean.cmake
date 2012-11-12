FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C25_GlobalPosition/msg"
  "../src/C25_GlobalPosition/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/C25.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_C25.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
