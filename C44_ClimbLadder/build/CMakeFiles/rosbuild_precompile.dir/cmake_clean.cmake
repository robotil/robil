FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C44_ClimbLadder/msg"
  "../src/C44_ClimbLadder/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/rosbuild_precompile"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rosbuild_precompile.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
