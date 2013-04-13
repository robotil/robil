FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C23_ObjectRecognition/msg"
  "../src/C23_ObjectRecognition/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_genaction_msgs"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genaction_msgs.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
