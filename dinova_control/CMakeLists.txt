cmake_minimum_required(VERSION 3.0.2)
project(dinova_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dinova_control
  CATKIN_DEPENDS 
    rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
  scripts/kinova_driver.py
  scripts/dinova_state_publisher.py
  scripts/dingo_state_publisher.py
  scripts/generate_fk.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

set(SRC_PATH ${PROJECT_SOURCE_DIR}/src/autogen)

file(GLOB SRC_FILES "${SRC_PATH}/*.cpp")

foreach(SRC_FILE ${SRC_FILES})
    get_filename_component(LIB_NAME ${SRC_FILE} NAME_WE)
    
    add_library(${LIB_NAME} SHARED ${SRC_FILE})

endforeach()
