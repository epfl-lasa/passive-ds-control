cmake_minimum_required(VERSION 2.8.3)
project(passive_ds_control)

#set(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_BUILD_TYPE RELEASE)

set(CMAKE_CXX_FLAGS "-O3")
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS)

add_definitions(-DUSE_DOUBLE_PRECISION)


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
  LIBRARIES passive_ds_control cascade_ds_control openloop_ds_control
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS})

#this step is required to have the files how up in qtcreator
file(GLOB HEADERS "include/*.h")
#file(GLOB HEADERS "include/locally_modulated_ds/*.h")

## Declare a cpp library
add_library(passive_ds_control
  src/passive_ds_controller.cpp ${HEADERS}
)

add_library(cascade_ds_control
  src/cascade_ds_controller.cpp ${HEADERS}
)

add_library(openloop_ds_control
  src/openloop_ds_controller.cpp
)
## Declare a cpp executable
#add_executable(passive_ds_test src/main.cpp)
#add_executable(cascade_ds_test src/cascade_main.cpp)
#add_executable(openloop_test src/openloop_test.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(passive_ds_control_node passive_ds_control_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(passive_ds_control_node
#   ${catkin_LIBRARIES}
# )
#target_link_libraries(passive_ds_test passive_ds_control ${catkin_LIBRARIES})
#target_link_libraries(cascade_ds_test cascade_ds_control ${catkin_LIBRARIES})
#target_link_libraries(openloop_test openloop_ds_control ${catkin_LIBRARIES})
