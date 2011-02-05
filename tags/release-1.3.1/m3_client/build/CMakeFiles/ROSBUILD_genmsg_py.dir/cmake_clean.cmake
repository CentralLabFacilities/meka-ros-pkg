FILE(REMOVE_RECURSE
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/m3_client/msg/__init__.py"
  "../src/m3_client/msg/_M3BaseStatus.py"
  "../src/m3_client/msg/_M3OmnibaseJoy.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
