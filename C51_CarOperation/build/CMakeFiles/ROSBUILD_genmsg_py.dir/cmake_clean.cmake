FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C51_CarOperation/msg"
  "../src/C51_CarOperation/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C51_CarOperation/msg/__init__.py"
  "../src/C51_CarOperation/msg/_C0C51_TRA.py"
  "../src/C51_CarOperation/msg/_C0C51_ST.py"
  "../src/C51_CarOperation/msg/_C0C51_CL.py"
  "../src/C51_CarOperation/msg/_C51C0_OPO.py"
  "../src/C51_CarOperation/msg/_C0C51_PAR.py"
  "../src/C51_CarOperation/msg/_C51C0_NOR.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
