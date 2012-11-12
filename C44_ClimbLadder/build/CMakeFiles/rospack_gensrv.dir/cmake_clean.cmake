FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C44_ClimbLadder/msg"
  "../src/C44_ClimbLadder/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/rospack_gensrv"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/rospack_gensrv.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
