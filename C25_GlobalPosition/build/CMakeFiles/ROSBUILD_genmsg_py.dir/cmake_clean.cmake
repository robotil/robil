FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C25_GlobalPosition/msg"
  "../src/C25_GlobalPosition/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C25_GlobalPosition/msg/__init__.py"
  "../src/C25_GlobalPosition/msg/_C0C25_AZI.py"
  "../src/C25_GlobalPosition/msg/_C0C25_CAM.py"
  "../src/C25_GlobalPosition/msg/_C25C0_OPO.py"
  "../src/C25_GlobalPosition/msg/_C25C0_ROP.py"
  "../src/C25_GlobalPosition/msg/_C0C25_LAZ.py"
  "../src/C25_GlobalPosition/msg/_UTM.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
