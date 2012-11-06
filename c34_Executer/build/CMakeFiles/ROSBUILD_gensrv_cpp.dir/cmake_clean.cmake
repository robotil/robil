FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/c34_Executer/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_cpp"
  "../srv_gen/cpp/include/c34_Executer/stop.h"
  "../srv_gen/cpp/include/c34_Executer/help_msg.h"
  "../srv_gen/cpp/include/c34_Executer/pwd.h"
  "../srv_gen/cpp/include/c34_Executer/show_table_msg.h"
  "../srv_gen/cpp/include/c34_Executer/run.h"
  "../srv_gen/cpp/include/c34_Executer/resume.h"
  "../srv_gen/cpp/include/c34_Executer/lookup.h"
  "../srv_gen/cpp/include/c34_Executer/step.h"
  "../srv_gen/cpp/include/c34_Executer/btstack.h"
  "../srv_gen/cpp/include/c34_Executer/cd.h"
  "../srv_gen/cpp/include/c34_Executer/ls.h"
  "../srv_gen/cpp/include/c34_Executer/pause.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
