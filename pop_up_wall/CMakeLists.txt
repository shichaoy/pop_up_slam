cmake_minimum_required(VERSION 2.8.3)
project(pop_up_wall)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  cmake_modules
  cv_bridge
  pcl_conversions
  roscpp
  sensor_msgs
  tictoc_profiler  
  line_lbd
)
#lineseg_detect
## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen REQUIRED)
find_package(PythonLibs 2.7.6 REQUIRED) 

message(STATUS "Found OpencvDirs-----------------------------")
message(STATUS ${OpenCV_INCLUDE_DIRS})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules roslib cv_bridge pcl_conversions roscpp sensor_msgs tictoc_profiler line_lbd
  #DEPENDS system_lib
)

include_directories(
  include
#   third_party
  ${catkin_INCLUDE_DIRS}
  include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS} ${PYTHON_INCLUDE_DIRS}
)

set(CMAKE_BUILD_TYPE Release)


add_library(matrix_utils_lib   libs/matrix_utils.cpp)

# most important. this name should match the libname in catkin_package(...)
add_library(${PROJECT_NAME}   libs/popup_plane.cpp libs/matrix_utils.cpp libs/select_edge.cpp)	

#target_link_libraries(matrix_utils_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${OpenCV_LIBRARIES} ${EIGEN_LIBRARIES})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} python2.7 boost_python-py27)

## Add cmake target dependencies of the library 
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(pop_up_wall ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
#add_executable(pop_up_detail_node src/popup_plane_detail_vertex.cpp)
# add_executable(pop_up_detail_node src/popup_plane_detail_edge.cpp)
add_executable(pop_up_main_node src/popup_plane_main.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(pop_up_wall_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(pop_up_detail_node
#     matrix_utils_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${EIGEN_LIBRARIES} python2.7 boost_python-py27
# )
target_link_libraries(pop_up_main_node
    ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES}  ${EIGEN_LIBRARIES} ${PROJECT_NAME}
)

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
message("---------------------------------------------")
message(${PROJECT_NAME} " Configuration Summary")
message("Build type:                    ${CMAKE_BUILD_TYPE}")
message("---------------------------------------------")
