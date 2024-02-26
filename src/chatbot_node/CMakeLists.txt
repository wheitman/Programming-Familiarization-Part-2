# Package:   steward_msgs
# Filename:  CMakeLists.txt
# Author:    Will Heitman
# Email:     w@heit.mn
# Project:   Arbor Robotics at CMU, 2024
# License:   MIT License

cmake_minimum_required(VERSION 3.5)
get_filename_component(directory_name ${CMAKE_CURRENT_SOURCE_DIR} NAME)
project(${directory_name})

find_package(ament_cmake_auto REQUIRED) # The greatest package in history
ament_auto_find_build_dependencies() # Find dependencies listed in package.xml

# Find message files
file(GLOB_RECURSE msg_filenames RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "msg/*.msg") # Find all message files
file(GLOB_RECURSE srv_filenames RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "srv/*.srv") # Find all srv files
rosidl_generate_interfaces(
  ${PROJECT_NAME}
  ${msg_filenames}
  ${srv_filenames}
  DEPENDENCIES
  ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)

# Build a shared library
file(GLOB_RECURSE source_filenames "src/*.cpp") # Find all source files in src/
if(source_filenames) # If we have any source files
  ament_auto_add_library(${PROJECT_NAME}_lib SHARED DIRECTORY src) # Then make a library
endif()

# Build all executables
file(GLOB_RECURSE executable_filenames "exe/*.cpp") # Get all source files in exe/
foreach(filename ${executable_filenames}) # Iterate over them
  get_filename_component(executable_name ${filename} NAME_WE) # Pull the name with no extension
  ament_auto_add_executable(${executable_name} ${filename}) # Add an executable by that name
endforeach()

ament_auto_package()
