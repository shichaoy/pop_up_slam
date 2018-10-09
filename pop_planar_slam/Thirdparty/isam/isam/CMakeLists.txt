cmake_minimum_required (VERSION 2.6)

link_libraries(isamlib)

# main iSAM executable (for processing files, timing, includes GUI, LCM interface)
set (ISAM_SOURCES isam.cpp Loader.cpp)
if(USE_LCM)
  set (ISAM_SOURCES ${ISAM_SOURCES} Lcm.cpp)
  add_subdirectory(lcmtypes)
endif(USE_LCM)
if(USE_GUI)
  set (ISAM_SOURCES ${ISAM_SOURCES} Collections.cpp Viewer.cpp)
endif(USE_GUI)

add_executable(isam ${ISAM_SOURCES})
if (PROFILE)
  set_target_properties(isam PROPERTIES LINK_FLAGS "-pg")
endif (PROFILE)
if(USE_LCM)
  target_link_libraries(isam lcm lcmtypes)
endif(USE_LCM) 
if(USE_GUI)
  find_package(OpenGL REQUIRED)
  find_package(SDL REQUIRED)
  include_directories(${GL_INCLUDE} ${SDL_INCLUDE_DIR})
  target_link_libraries(isam ${OPENGL_LIBRARY} ${SDL_LIBRARY})
endif(USE_GUI)

install(TARGETS isam
  RUNTIME DESTINATION bin
)
