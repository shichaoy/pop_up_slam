cmake_minimum_required(VERSION 2.8.3)
project(odom_tf_utils)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  tf
)

## System dependencies are found with CMake's conventions

# catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES
  CATKIN_DEPENDS nav_msgs roscpp tf
#  DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(odom_to_tf src/odom_to_tf.cpp)
target_link_libraries(odom_to_tf
  ${catkin_LIBRARIES}
)

add_executable(odom_ned_tf src/odom_ned_tf.cpp)
target_link_libraries(odom_ned_tf
  ${catkin_LIBRARIES}
)

add_executable(tf_to_odom src/tf_to_odom.cpp)
target_link_libraries(tf_to_odom
  ${catkin_LIBRARIES}
)

install(TARGETS odom_to_tf tf_to_odom odom_ned_tf
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

