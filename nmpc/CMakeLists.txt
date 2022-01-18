cmake_minimum_required(VERSION 3.0.2)
project(nmpc)

## Compile as C++14
add_compile_options(-std=c++14)

################
# Dependencies #
################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  state_estimation
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
  CATKIN_DEPENDS roscpp std_msgs state_estimation
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

add_executable(px4_nmpc_node
  src/px4_nmpc_node.cc
  src/px4_nmpc.cc
)

add_dependencies(px4_nmpc_node
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(px4_nmpc_node
  ${catkin_LIBRARIES}
  ${acados_LIBRARIES}
  ${nmpc_generated_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_nmpc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
