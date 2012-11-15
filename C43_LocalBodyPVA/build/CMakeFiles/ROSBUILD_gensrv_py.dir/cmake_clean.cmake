FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C43_LocalBodyPVA/msg"
  "../src/C43_LocalBodyPVA/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/C43_LocalBodyPVA/srv/__init__.py"
  "../src/C43_LocalBodyPVA/srv/_C43.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
