FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C22_GroundRecognitionAndMapping/msg"
  "../src/C22_GroundRecognitionAndMapping/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C22_GroundRecognitionAndMapping/msg/__init__.py"
  "../src/C22_GroundRecognitionAndMapping/msg/_C0C22_CAM.py"
  "../src/C22_GroundRecognitionAndMapping/msg/_C0C22_SAF.py"
  "../src/C22_GroundRecognitionAndMapping/msg/_C0C22_AZI.py"
  "../src/C22_GroundRecognitionAndMapping/msg/_C22C0_PATH.py"
  "../src/C22_GroundRecognitionAndMapping/msg/_C0C22_LAZ.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
