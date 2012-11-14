FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C43_LocalBodyPVA/msg"
  "../src/C43_LocalBodyPVA/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C43_LocalBodyPVA/msg/__init__.py"
  "../src/C43_LocalBodyPVA/msg/_C43C0_JPVA.py"
  "../src/C43_LocalBodyPVA/msg/_C0C43_SJ.py"
  "../src/C43_LocalBodyPVA/msg/_C0C43_SL.py"
  "../src/C43_LocalBodyPVA/msg/_C43C0_LPVA.py"
  "../src/C43_LocalBodyPVA/msg/_C0C43_SI.py"
  "../src/C43_LocalBodyPVA/msg/_C0C43_SLI.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
