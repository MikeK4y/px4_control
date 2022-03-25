cmake_minimum_required(VERSION 3.0.2)
project(nmpc)

## Compile as C++14
add_compile_options(-std=c++14)

# Option to get acados NMPC time information
option(TRACK_ACADOS_TIME "Track acados OCP solution time" ON)

if(TRACK_ACADOS_TIME)
  add_definitions(-DTRACK_TIME)
endif()

################
# Dependencies #
################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  px4_control_msgs
)

# Acados
set(acados_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/../acados/include")
set(acados_LIBRARIES "${PROJECT_SOURCE_DIR}/../acados/lib/libacados.so")

# NMPC generated
set(nmpc_generated_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/scripts/c_generated_code")
set(nmpc_generated_LIBRARIES "${PROJECT_SOURCE_DIR}/scripts/c_generated_code/libacados_ocp_solver_drone_w_disturbances.so")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES nmpc
  CATKIN_DEPENDS roscpp std_msgs px4_control_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${acados_INCLUDE_DIRS}
  ${acados_INCLUDE_DIRS}/blasfeo/include
  ${acados_INCLUDE_DIRS}/hpipm/include
  ${nmpc_generated_INCLUDE_DIRS}
)

## Runnables
add_library(acados_nmpc
  src/acados_nmpc.cc
)

add_executable(px4_nmpc_node
  src/px4_control_node.cc
  src/px4_control.cc
)

add_executable(trajectory_generation_node
  src/trajectory_generation.cc
)

## Dependencies
add_dependencies(px4_nmpc_node
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(trajectory_generation_node
  ${catkin_EXPORTED_TARGETS}
)

## Links
target_link_libraries(acados_nmpc
  ${acados_LIBRARIES}
  ${nmpc_generated_LIBRARIES}
)

target_link_libraries(px4_nmpc_node
  ${catkin_LIBRARIES}
  acados_nmpc
)

target_link_libraries(trajectory_generation_node
  ${catkin_LIBRARIES}
  acados_nmpc
)

#############
## Install ##
#############