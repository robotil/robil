FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/c34_Executer/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/c34_Executer/srv/__init__.py"
  "../src/c34_Executer/srv/_stop.py"
  "../src/c34_Executer/srv/_help_msg.py"
  "../src/c34_Executer/srv/_pwd.py"
  "../src/c34_Executer/srv/_show_table_msg.py"
  "../src/c34_Executer/srv/_run.py"
  "../src/c34_Executer/srv/_resume.py"
  "../src/c34_Executer/srv/_lookup.py"
  "../src/c34_Executer/srv/_step.py"
  "../src/c34_Executer/srv/_btstack.py"
  "../src/c34_Executer/srv/_cd.py"
  "../src/c34_Executer/srv/_ls.py"
  "../src/c34_Executer/srv/_pause.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
