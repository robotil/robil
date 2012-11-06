FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/Executer/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/Executer/srv/__init__.py"
  "../src/Executer/srv/_stop.py"
  "../src/Executer/srv/_help_msg.py"
  "../src/Executer/srv/_pwd.py"
  "../src/Executer/srv/_show_table_msg.py"
  "../src/Executer/srv/_run.py"
  "../src/Executer/srv/_resume.py"
  "../src/Executer/srv/_lookup.py"
  "../src/Executer/srv/_step.py"
  "../src/Executer/srv/_btstack.py"
  "../src/Executer/srv/_cd.py"
  "../src/Executer/srv/_ls.py"
  "../src/Executer/srv/_pause.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
