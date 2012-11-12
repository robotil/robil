FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C44_ClimbLadder/msg"
  "../src/C44_ClimbLadder/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C44_ClimbLadder/msg/__init__.py"
  "../src/C44_ClimbLadder/msg/_C23C44_DIS.py"
  "../src/C44_ClimbLadder/msg/_C44C0_CLS.py"
  "../src/C44_ClimbLadder/msg/_C23C44_LDIM.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
