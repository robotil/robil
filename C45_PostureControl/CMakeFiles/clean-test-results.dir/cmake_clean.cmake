FILE(REMOVE_RECURSE
  "msg_gen"
  "srv_gen"
  "src/C45_PostureControl/msg"
  "src/C45_PostureControl/srv"
  "msg_gen"
  "srv_gen"
  "CMakeFiles/clean-test-results"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/clean-test-results.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
