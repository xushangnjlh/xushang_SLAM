FILE(REMOVE_RECURSE
  "CMakeFiles/mytest.dir/src/mytest.cpp.o"
  "../bin/mytest.pdb"
  "../bin/mytest"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang CXX)
  INCLUDE(CMakeFiles/mytest.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
