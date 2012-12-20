FILE(REMOVE_RECURSE
  "../srv_gen"
  "../src/C34_Executer/srv"
  "../srv_gen"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/C34_Executer/srv/__init__.py"
  "../src/C34_Executer/srv/_stop.py"
  "../src/C34_Executer/srv/_help_msg.py"
  "../src/C34_Executer/srv/_pwd.py"
  "../src/C34_Executer/srv/_show_table_msg.py"
  "../src/C34_Executer/srv/_run.py"
  "../src/C34_Executer/srv/_resume.py"
  "../src/C34_Executer/srv/_lookup.py"
  "../src/C34_Executer/srv/_whoIsRunning.py"
  "../src/C34_Executer/srv/_step.py"
  "../src/C34_Executer/srv/_read_file.py"
  "../src/C34_Executer/srv/_btstack.py"
  "../src/C34_Executer/srv/_cd.py"
  "../src/C34_Executer/srv/_ls.py"
  "../src/C34_Executer/srv/_version.py"
  "../src/C34_Executer/srv/_pause.py"
  "../src/C34_Executer/srv/_save_file.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
