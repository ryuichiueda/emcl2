cmake_minimum_required(VERSION 3.0.2)
project(emcl2)

set(CMAKE_CXX_FLAGS "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf2
  tf2_ros
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mcl
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} )

add_library(Mcl_emcl2 src/Mcl.cpp)
add_library(ExpResetMcl_emcl2 src/ExpResetMcl.cpp)
add_library(ExpResetMcl2_emcl2 src/ExpResetMcl2.cpp)
add_library(Particle_emcl2 src/Particle.cpp)
add_library(OdomModel_emcl2 src/OdomModel.cpp)
add_library(Pose_emcl2 src/Pose.cpp)
add_library(Scan_emcl2 src/Scan.cpp)
add_library(LikelihoodFieldMap_emcl2 src/LikelihoodFieldMap.cpp)

#add_executable(mcl_node src/mcl_node.cpp)
#add_executable(emcl_node src/emcl_node.cpp)
add_executable(emcl2_node src/emcl2_node.cpp)

#add_dependencies(mcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(emcl_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(emcl2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#target_link_libraries(mcl_node Mcl Particle Pose Scan OdomModel LikelihoodFieldMap ${catkin_LIBRARIES})
#target_link_libraries(emcl_node Mcl ExpResetMcl Particle Pose Scan OdomModel LikelihoodFieldMap ${catkin_LIBRARIES})
target_link_libraries(emcl2_node Mcl_emcl2 ExpResetMcl2_emcl2 Particle_emcl2 Pose_emcl2 Scan_emcl2 OdomModel_emcl2 LikelihoodFieldMap_emcl2 ${catkin_LIBRARIES})

