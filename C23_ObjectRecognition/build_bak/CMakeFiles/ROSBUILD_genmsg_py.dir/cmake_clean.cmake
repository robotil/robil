FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C23_ObjectRecognition/msg"
  "../src/C23_ObjectRecognition/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/C23_ObjectRecognition/msg/__init__.py"
  "../src/C23_ObjectRecognition/msg/_C23C0_ODIM.py"
  "../src/C23_ObjectRecognition/msg/_C23C0_OPO.py"
  "../src/C23_ObjectRecognition/msg/_C0C23_SAR.py"
  "../src/C23_ObjectRecognition/msg/_C0C23_SEC.py"
  "../src/C23_ObjectRecognition/msg/_C0C23_SEOB.py"
  "../src/C23_ObjectRecognition/msg/_C23C0_OD.py"
  "../src/C23_ObjectRecognition/msg/_TBD.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
