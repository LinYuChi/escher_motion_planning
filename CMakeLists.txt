cmake_minimum_required (VERSION 2.6.0)
project(escher_motion_planning)
find_package(OpenRAVE 0.9 REQUIRED)
set(CMAKE_CXX_STANDARD 11)

if( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )
  add_definitions("-fno-strict-aliasing -Wall")
endif( CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX )

# optional in case boost is used
find_package(Boost ${OpenRAVE_Boost_VERSION} EXACT)

include_directories(${OpenRAVE_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${OpenRAVE_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(escher_motion_planning SHARED escher_motion_planning.cpp)
add_library(structures SHARED structures.cpp)
add_library(environment_handler SHARED environment_handler.cpp)
set_target_properties(escher_motion_planning PROPERTIES COMPILE_FLAGS "${OpenRAVE_CXX_FLAGS}" LINK_FLAGS "${OpenRAVE_LINK_FLAGS}")
target_link_libraries(escher_motion_planning environment_handler ${OpenRAVE_LIBRARIES})
target_link_libraries(environment_handler structures ${OpenRAVE_LIBRARIES})
install(TARGETS escher_motion_planning DESTINATION .)
