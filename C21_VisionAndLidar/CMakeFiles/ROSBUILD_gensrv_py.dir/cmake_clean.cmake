FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/C21_VisionAndLidar/msg"
  "src/C21_VisionAndLidar/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "src/C21_VisionAndLidar/srv/__init__.py"
  "src/C21_VisionAndLidar/srv/_C21.py"
  "src/C21_VisionAndLidar/srv/_C21_Pan.py"
  "src/C21_VisionAndLidar/srv/_C21_Pic.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
