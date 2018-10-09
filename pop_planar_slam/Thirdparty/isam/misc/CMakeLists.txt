cmake_minimum_required (VERSION 2.6)

link_libraries(isamlib)

# generate a target for each .cpp file in this directory
file(GLOB misc RELATIVE "${PROJECT_SOURCE_DIR}/misc" "*.cpp")
add_custom_target(misc)
foreach(file ${misc})
  string(REPLACE ".cpp" "" file ${file})
  add_executable(${file} EXCLUDE_FROM_ALL ${file}.cpp)
  add_dependencies(misc ${file})
endforeach(file)
