FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C24_ObstacleDetection/msg"
  "../src/C24_ObstacleDetection/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/rospack_genmsg_all"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_genmsg_all.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
