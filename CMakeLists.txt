cmake_minimum_required(VERSION 2.8.3)
project(simple_scan_matcher)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roscpp
  tf
  sensor_msgs
  ceres_solver
  dynamic_reconfigure
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

generate_dynamic_reconfigure_options(
    cfg/scan_matcher.cfg
)

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
  LIBRARIES simple_scan_matcher
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS BOOST
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/similitude.cpp
   src/scan_matcher.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(simple_scan_matcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
add_executable(scan_matcher_node src/scan_matcher_node.cpp)
add_dependencies(scan_matcher_node ${PROJECT_NAME}_gencfg simple_scan_matcher_generate_messages_cpp)
target_link_libraries(scan_matcher_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${BOOST_LIBRARIES}
)

## Declare a C++ executable
add_executable(test_sim_est sample/test_similitude.cpp)
target_link_libraries(test_sim_est
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${BOOST_LIBRARIES}
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(simple_scan_matcher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(simple_scan_matcher_node
#   ${catkin_LIBRARIES}
#   ${BOOST_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS simple_scan_matcher simple_scan_matcher_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_simple_scan_matcher.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
