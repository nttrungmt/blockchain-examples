project(blockchain_voting_loop_functions)
include_directories("${CMAKE_SOURCE_DIR}"
	"${CMAKE_SOURCE_DIR}/controllers"
#	"${footbot_diffusion_INCLUDE_DIRS}"
	"${PROJECT_SOURCE_DIR}")
message(STATUS "${CMAKE_SOURCE_DIR}")

# If Qt+OpenGL dependencies were found, descend into these directories
if(ARGOS_COMPILE_QTOPENGL)
  add_library(blockchain_voting_loop_functions MODULE 
    blockchain_loop_functions.h
    id_qtuser_functions.h
    blockchain_loop_functions.cpp
    id_qtuser_functions.cpp)

  target_link_libraries(blockchain_voting_loop_functions
    footbot_diffusion
    argos3core_simulator
    argos3plugin_simulator_entities
    argos3plugin_simulator_footbot
    argos3plugin_simulator_qtopengl
    ${ARGOS_QTOPENGL_LIBRARIES})
endif(ARGOS_COMPILE_QTOPENGL)