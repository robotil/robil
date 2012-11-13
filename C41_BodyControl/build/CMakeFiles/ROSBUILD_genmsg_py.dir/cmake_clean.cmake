FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C41_BodyControl/msg"
  "../src/C41_BodyControl/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C41_BodyControl/msg/__init__.py"
  "../src/C41_BodyControl/msg/_C0C41_PVA.py"
  "../src/C41_BodyControl/msg/_C0C41_WM.py"
  "../src/C41_BodyControl/msg/_C41C0_APVA.py"
  "../src/C41_BodyControl/msg/_C0C41_TC.py"
  "../src/C41_BodyControl/msg/_C0C41_LOAD.py"
  "../src/C41_BodyControl/msg/_C41C0_AT.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
