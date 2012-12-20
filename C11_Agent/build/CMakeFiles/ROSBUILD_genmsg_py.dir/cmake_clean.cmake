FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C11_Agent/msg"
  "../src/C11_Agent/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C11_Agent/msg/__init__.py"
  "../src/C11_Agent/msg/_C34C11_STT.py"
  "../src/C11_Agent/msg/_C23C11_OSM.py"
  "../src/C11_Agent/msg/_C11C23_OBP.py"
  "../src/C11_Agent/msg/_C32C11_PATH.py"
  "../src/C11_Agent/msg/_C11C23_OBM.py"
  "../src/C11_Agent/msg/_C11C24_OSM.py"
  "../src/C11_Agent/msg/_D3SPACE.py"
  "../src/C11_Agent/msg/_C11C32_PATH.py"
  "../src/C11_Agent/msg/_coord.py"
  "../src/C11_Agent/msg/_C24C11_OSM.py"
  "../src/C11_Agent/msg/_C11C24_OSP.py"
  "../src/C11_Agent/msg/_pathLocation.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
