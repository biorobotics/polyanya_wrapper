cmake_minimum_required(VERSION 3.0)
project(polyanya_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-fopenmp)

find_package(pybind11 REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(
  include
)

pybind11_add_module(polyanya_wrapper
  src/bind_polyanya_wrapper.cpp
  src/polyanya_wrapper.cpp
  $ENV{HOME}/polyanya/anyangle/polyanya/structs/mesh.cpp
  $ENV{HOME}/polyanya/anyangle/polyanya/helpers/geometry.cpp
  $ENV{HOME}/polyanya/anyangle/polyanya/helpers/timer.cpp
  $ENV{HOME}/polyanya/anyangle/polyanya/helpers/scenario.cpp
  $ENV{HOME}/polyanya/anyangle/polyanya/search/expansion.cpp
  $ENV{HOME}/polyanya/anyangle/polyanya/search/searchinstance.cpp
)
target_include_directories(polyanya_wrapper PRIVATE
  $ENV{HOME}/polyanya/anyangle/polyanya/structs
  $ENV{HOME}/polyanya/anyangle/polyanya/helpers
  $ENV{HOME}/polyanya/anyangle/polyanya/search
)
