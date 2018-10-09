cmake_minimum_required (VERSION 2.6)

find_library(LIB_LCM lcm)

FILE(GLOB LCM_FILES "*.lcm")

string(REPLACE ".lcm" ".c" LCM_SOURCES "${LCM_FILES}")
string(REPLACE ".lcm" ".h" LCM_HEADERS "${LCM_FILES}")
string(REPLACE ".lcm" "" LCM_TYPES "${LCM_FILES}")

foreach (LCM_TYPE ${LCM_TYPES})
  add_custom_command (
    OUTPUT "${LCM_TYPE}.c" "${LCM_TYPE}.h"
    COMMAND lcm-gen "${LCM_TYPE}.lcm" --lazy -c
    DEPENDS "${LCM_TYPE}.lcm"
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    )
endforeach (LCM_TYPE)

set_source_files_properties(${LCM_SOURCES} ${LCM_HEADERS} PROPERTIES GENERATED true)
add_library (lcmtypes ${LCM_SOURCES} ${LCM_HEADERS})
target_link_libraries (lcmtypes ${LIB_LCM})
