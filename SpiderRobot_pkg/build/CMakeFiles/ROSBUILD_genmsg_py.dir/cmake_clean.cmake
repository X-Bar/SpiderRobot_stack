FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/SpiderRobot_pkg/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/SpiderRobot_pkg/msg/__init__.py"
  "../src/SpiderRobot_pkg/msg/_My2Num.py"
  "../src/SpiderRobot_pkg/msg/_MyChar.py"
  "../src/SpiderRobot_pkg/msg/_MyArray.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
