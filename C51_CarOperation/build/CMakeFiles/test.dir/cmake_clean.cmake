FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../src/C51_CarOperation/msg"
  "../src/C51_CarOperation/srv"
  "../msg_gen"
  "../srv_gen"
  "CMakeFiles/test"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/test.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
