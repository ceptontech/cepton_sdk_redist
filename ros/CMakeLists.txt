cmake_minimum_required(VERSION 3.1)

set(CEPTON_ROS_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}")
set(CEPTON_SDK_SOURCE_DIR "${CEPTON_ROS_SOURCE_DIR}/third_party/cepton_sdk")
list(APPEND CMAKE_MODULE_PATH "${CEPTON_SDK_SOURCE_DIR}/cmake")

include("CeptonVersion")
parse_version_file("${CMAKE_CURRENT_LIST_DIR}/VERSION" CEPTON_ROS)
project(
  cepton_ros
  VERSION ${CEPTON_ROS_VERSION}
  LANGUAGES C CXX)

# ------------------------------------------------------------------------------
# Variables
# ------------------------------------------------------------------------------
execute_process(
  COMMAND uname -m
  OUTPUT_VARIABLE MACHINE_ARCH
  OUTPUT_STRIP_TRAILING_WHITESPACE)
set(OS_NAME "linux-${MACHINE_ARCH}")

# ------------------------------------------------------------------------------
# Compiler Flags
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 11)

# ------------------------------------------------------------------------------
# External libraries
# ------------------------------------------------------------------------------
# catkin
# ------
set(CEPTON_ROS_CATKIN_DEPENDS
    nodelet
    pcl_conversions
    pcl_ros
    pluginlib
    roscpp
    roslib
    std_msgs
    tf2_ros)
find_package(catkin REQUIRED COMPONENTS ${CEPTON_ROS_CATKIN_DEPENDS}
                                        message_generation)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

set(CEPTON_ROS_THIRD_PARTY_LIBRARIES "")

# cepton_sdk
# ----------
set(CEPTON_SDK_USE_STATIC_OPTION
    FALSE
    CACHE INTERNAL "" FORCE)
set(CEPTON_SDK_BINARY_DIR "${PROJECT_BINARY_DIR}/third_party/cepton_sdk")
add_subdirectory("${CEPTON_SDK_SOURCE_DIR}" "${CEPTON_SDK_BINARY_DIR}")
list(APPEND CEPTON_ROS_THIRD_PARTY_LIBRARIES cepton_sdk::cepton_sdk)

# ------------------------------------------------------------------------------
# Targets
# ------------------------------------------------------------------------------
add_message_files(FILES SensorInformation.msg)

generate_messages(DEPENDENCIES std_msgs)

set(CEPTON_ROS_LIBRARIES "")

add_library(
  cepton_ros
  "${CEPTON_ROS_SOURCE_DIR}/src/common.cpp"
  "${CEPTON_ROS_SOURCE_DIR}/src/driver_nodelet.cpp"
  "${CEPTON_ROS_SOURCE_DIR}/src/subscriber_nodelet.cpp")
list(APPEND CEPTON_ROS_LIBRARIES cepton_ros)

foreach(name IN LISTS CEPTON_ROS_LIBRARIES)
  target_include_directories(
    ${name} PUBLIC $<BUILD_INTERFACE:${CEPTON_ROS_SOURCE_DIR}/include>
                   $<INSTALL_INTERFACE:include>)
  target_link_libraries(${name} ${catkin_LIBRARIES}
                        ${CEPTON_ROS_THIRD_PARTY_LIBRARIES})
  add_dependencies(${name} ${catkin_EXPORTED_TARGETS}
                   ${cepton_ros_EXPORTED_TARGETS})
endforeach()

# ------------------------------------------------------------------------------
# Install
# ------------------------------------------------------------------------------
catkin_package(
  CATKIN_DEPENDS
  ${CEPTON_ROS_CATKIN_DEPENDS}
  message_runtime
  DEPENDS
  INCLUDE_DIRS
  "include"
  LIBRARIES
  ${CEPTON_ROS_THIRD_PARTY_LIBRARIES}
  ${CEPTON_ROS_LIBRARIES})

install(DIRECTORY "include/cepton_ros/"
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(
  TARGETS ${CEPTON_ROS_LIBRARIES}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
