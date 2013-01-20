FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "msg_gen"
  "srv_gen"
  "src/c21_Vision_and_Lidar/msg"
  "src/c21_Vision_and_Lidar/srv"
  "CMakeFiles/rosbuild_premsgsrvgen"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rosbuild_premsgsrvgen.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
