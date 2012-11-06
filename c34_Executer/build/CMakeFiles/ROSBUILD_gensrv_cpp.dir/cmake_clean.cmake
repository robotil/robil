FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/Executer/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/Executer/stop.h"
  "../srv_gen/cpp/include/Executer/help_msg.h"
  "../srv_gen/cpp/include/Executer/pwd.h"
  "../srv_gen/cpp/include/Executer/show_table_msg.h"
  "../srv_gen/cpp/include/Executer/run.h"
  "../srv_gen/cpp/include/Executer/resume.h"
  "../srv_gen/cpp/include/Executer/lookup.h"
  "../srv_gen/cpp/include/Executer/step.h"
  "../srv_gen/cpp/include/Executer/btstack.h"
  "../srv_gen/cpp/include/Executer/cd.h"
  "../srv_gen/cpp/include/Executer/ls.h"
  "../srv_gen/cpp/include/Executer/pause.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
