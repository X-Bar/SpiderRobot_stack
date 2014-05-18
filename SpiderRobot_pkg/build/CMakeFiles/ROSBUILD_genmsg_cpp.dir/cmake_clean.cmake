FILE(REMOVE_RECURSE
  "../src/SpiderRobot_pkg/msg"
  "../msg_gen"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/SpiderRobot_pkg/My2Num.h"
  "../msg_gen/cpp/include/SpiderRobot_pkg/MyChar.h"
  "../msg_gen/cpp/include/SpiderRobot_pkg/MyArray.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
