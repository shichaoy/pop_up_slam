cmake_minimum_required (VERSION 2.6)

link_libraries(isamlib)

# generate a target for each .cpp file in this directory
file(GLOB examples RELATIVE "${PROJECT_SOURCE_DIR}/examples" "*.cpp")
add_custom_target(examples)
foreach(example ${examples})
  string(REPLACE ".cpp" "" example ${example})
  add_executable(${example} EXCLUDE_FROM_ALL ${example}.cpp)
  add_dependencies(examples ${example})
endforeach(example)
