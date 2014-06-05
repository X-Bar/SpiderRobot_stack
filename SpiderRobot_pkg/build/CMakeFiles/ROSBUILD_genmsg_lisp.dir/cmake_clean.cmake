FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/SpiderRobot_pkg/msg"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/MYMSG.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MYMSG.lisp"
  "../msg_gen/lisp/My2Num.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_My2Num.lisp"
  "../msg_gen/lisp/MyChar.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MyChar.lisp"
  "../msg_gen/lisp/MyArray.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_MyArray.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
