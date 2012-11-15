FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C22_GroundRecognitionAndMapping/msg"
  "../src/C22_GroundRecognitionAndMapping/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/C22_GroundRecognitionAndMapping/srv/__init__.py"
  "../src/C22_GroundRecognitionAndMapping/srv/_C22.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
