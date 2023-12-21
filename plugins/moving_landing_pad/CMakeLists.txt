cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(moving_pad)

# Find packages

find_package(gazebo REQUIRED)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

# include appropriate directories
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

# Create libraries and executables

add_library(moving_pad SHARED moving_pad.cc)
target_link_libraries(moving_pad ${GAZEBO_LIBRARIES})

add_library(moving_pad_circle SHARED moving_pad_circle.cc)
target_link_libraries(moving_pad_circle ${GAZEBO_LIBRARIES})