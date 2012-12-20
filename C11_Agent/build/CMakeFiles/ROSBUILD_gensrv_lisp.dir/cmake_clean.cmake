FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C11_Agent/msg"
  "../src/C11_Agent/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_lisp"
  "../srv_gen/lisp/object_map.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_object_map.lisp"
  "../srv_gen/lisp/obstacle_map.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_obstacle_map.lisp"
  "../srv_gen/lisp/override_object_properties.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_override_object_properties.lisp"
  "../srv_gen/lisp/override_obstacle_properties.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_override_obstacle_properties.lisp"
  "../srv_gen/lisp/C11.lisp"
  "../srv_gen/lisp/_package.lisp"
  "../srv_gen/lisp/_package_C11.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
