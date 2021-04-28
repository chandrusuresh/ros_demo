cmake_minimum_required(VERSION 3.0.2)
project(ros_demo)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  geometry_msgs
  mav_msgs
  roscpp
  rotors_gazebo_plugins
  sensor_msgs
  xacro
  gazebo_plugins
)

find_package(Eigen3 REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(Protobuf REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)
# include(/home/kakul/catkin_ws/src/rotors_simulator/rotors_gazebo_plugins/CMakeLists.txt)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(/usr/include/gazebo-7/gazebo/msgs)

# ============================================== #
# =================== PROTOBUF ================= #
# ============================================== #

set(PROTOBUF_IMPORT_DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()
message(STATUS "PROTOBUF_IMPORT_DIRS = " ${PROTOBUF_IMPORT_DIRS})

# protobuf required for publishing/subscribing to Gazebo
# messages
# WARNING: THIS MUST BE RUN AFTER PROTOBUF_IMPORT_DIRS IS SETUP
find_package(Protobuf REQUIRED)
pkg_check_modules(PROTOBUF protobuf)

# # Protobuf version check
# if ("${PROTOBUF_VERSION}" VERSION_LESS "2.5.0")
#   message(FATAL_ERROR "protobuf version: ${PROTOBUF_VERSION} not compatible, must be >= 2.5.0")
# endif()

# # After running this block of code,
# # we should have something similar to
# # GAZEBO_MSG_INCLUDE_DIRS = /usr/include/gazebo-7/gazebo/msgs
# set(GAZEBO_MSG_INCLUDE_DIRS)
# foreach(ITR ${GAZEBO_INCLUDE_DIRS})
#   if(ITR MATCHES ".*gazebo-[0-9.]+$")
#     set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
#   endif()
# endforeach()

# Get lists of all .proto files in the msgs directory
file(GLOB msgs /home/kakul/catkin_ws/src/rotors_simulator/rotors_gazebo_plugins/msgs/*.proto)

# Finally, generate the .cpp files from the .proto files
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${msgs})
list(APPEND EXTLIBS ${PROTOBUF_LIBRARIES})

# Create a shared library of protobuf messages (.so extension on Linux platforms)
add_library(mav_msgs SHARED ${PROTO_SRCS})
target_link_libraries(mav_msgs ${PROTOBUF_LIBRARY} gazebo_msgs)


# This causes mav_msgs to be linked with every created library in this file from this
# point forward.
# NOTE: This is deprecated, should be using target_link_libraries instead
link_libraries(mav_msgs)

## Declare a C++ library
add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener ${catkin_EXPORTED_TARGETS})

add_executable(data_logger src/data_logger.cpp)
target_link_libraries(data_logger ${catkin_LIBRARIES})
add_dependencies(data_logger ${catkin_EXPORTED_TARGETS})

add_library(ros_demo_imu_plugin SHARED src/imu_plugin.cpp)
target_link_libraries(ros_demo_imu_plugin ${target_linking_LIBRARIES})
add_dependencies(ros_demo_imu_plugin ${catkin_EXPORTED_TARGETS})
list(APPEND targets_to_install ros_demo_imu_plugin)

####################
add_library(UKF src/UKF.cpp)
target_link_libraries(UKF ${catkin_LIBRARIES})
add_dependencies(UKF ${catkin_EXPORTED_TARGETS})

add_executable(state_estimation_node src/state_estimation_node.cpp)
add_dependencies(state_estimation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(state_estimation_node UKF ${catkin_LIBRARIES})#lee_position_controller ${catkin_LIBRARIES})

# install(TARGETS lee_position_controller roll_pitch_yawrate_thrust_controller
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
# )

install(TARGETS state_estimation_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

####################
foreach(dir include launch description)# models resource worlds)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

if (NOT NO_ROS)
  set(BIN_DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  set(LIB_DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
else()
  set(BIN_DESTINATION ${CMAKE_BINARY_DIR})
  set(LIB_DESTINATION ${CMAKE_BINARY_DIR})
endif()

# Install all targets that were defined above (some conditionally) and added to the
# variable targets_to_install
install(
  TARGETS
    ${targets_to_install}
  DESTINATION ${BIN_DESTINATION}
  LIBRARY DESTINATION ${LIB_DESTINATION}
)