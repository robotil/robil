FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/C11_OperatorControl/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/C11.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_C11.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
