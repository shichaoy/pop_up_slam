cmake_minimum_required(VERSION 2.8.3)
project(pop_planar_slam)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pop_up_wall  
  tictoc_profiler
  message_filters
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(PCL 1.6 REQUIRED)
find_package(Eigen REQUIRED)
find_package(PythonLibs 2.7 REQUIRED) 


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})

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
  #INCLUDE_DIRS include 
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules pop_up_wall tictoc_profiler message_filters
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS} 
    ${PYTHON_INCLUDE_DIRS}
)  


add_subdirectory(Thirdparty/isam)
#add_subdirectory(ThirdParty/DBoW2)


set(CMAKE_BUILD_TYPE Release)

message(STATUS "Build configuration: ${CMAKE_BUILD_TYPE}")


# cgal test
#add_executable(cgal_test cgal_utils/do_intersect.cpp)
#target_link_libraries(cgal_test CGAL CGAL_Core)


set(SRCS src/main_3d.cpp 
	src/Mapping.cpp 
	src/Map_plane.cpp 
	src/Frame.cpp
	src/isam_plane3d.cpp
	Thirdparty/ORB_SLAM/ORBextractor.cc 
	Thirdparty/ORB_SLAM/Converter.cc
)

# C++11
set_source_files_properties(${SRCS} PROPERTIES COMPILE_FLAGS -std=c++0x)
set(CMAKE_EXE_LINKER_FLAGS "-Wl,-rpath,. -L${CMAKE_CURRENT_BINARY_DIR}")
##########


# iSAM and Cholmod
include_directories(${CHOLMOD_INCLUDES} Thirdparty/isam/include)

add_executable(planar_test ${SRCS})
target_link_libraries(planar_test ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
target_link_libraries(planar_test isamlib -lz)
target_link_libraries(planar_test ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so)

