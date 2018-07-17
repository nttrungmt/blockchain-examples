project(footbot_diffusion)
include_directories(${CMAKE_SOURCE_DIR}/controllers)

#add_subdirectory(footbot_diffusion)
add_library(footbot_diffusion SHARED 
	footbot_diffusion.h 
	footbot_diffusion.cpp
	geth_wrapper.h
	geth_wrapper.cpp)
	
target_link_libraries(footbot_diffusion
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)

target_include_directories(footbot_diffusion PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>")

set(${PROJECT_NAME}_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)