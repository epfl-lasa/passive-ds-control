cmake_minimum_required(VERSION 3.9)

# Options. Turn on with 'cmake -Druntests=ON'.
option(runtests "Build all tests." OFF) # Makes boolean 'test' available.

project(passive_ds_control)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  	set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  	set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)

include_directories(
 	include
 	${Eigen3_INCLUDE_DIRS}
 )

file(GLOB HEADERS "include/passive_ds_control/*.hpp")

set(CORE_SOURCES
	src/DSController.cpp
	src/PassiveDSController.cpp
	src/CascadeDSController.cpp
	src/OpenloopDSController.cpp
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED 
	${CORE_SOURCES}
	${HEADERS}
)

install(DIRECTORY include/
  	DESTINATION include)

install(TARGETS ${PROJECT_NAME}
  	ARCHIVE DESTINATION lib
  	LIBRARY DESTINATION lib
  	RUNTIME DESTINATION bin)