FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C11_Agent/msg"
  "../src/C11_Agent/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/C11_Agent/srv/__init__.py"
  "../src/C11_Agent/srv/_object_map.py"
  "../src/C11_Agent/srv/_obstacle_map.py"
  "../src/C11_Agent/srv/_override_object_properties.py"
  "../src/C11_Agent/srv/_override_obstacle_properties.py"
  "../src/C11_Agent/srv/_C11.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
