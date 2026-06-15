cmake_minimum_required(VERSION 3.0)
project(polyanya_wrapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-fopenmp)

find_package(pybind11 REQUIRED)
find_package(OpenMP REQUIRED)
find_package(CGAL REQUIRED)

include_directories(
  include
)

add_library(visibility SHARED
  src/visibility_wrapper.cpp
)
target_link_libraries(visibility PUBLIC CGAL::CGAL)

pybind11_add_module(visibility_bindings
  src/bind_visibility.cpp
)
target_link_libraries(visibility_bindings PRIVATE visibility)

pybind11_add_module(polyanya_wrapper
  src/bind_polyanya_wrapper.cpp
  src/polyanya_wrapper.cpp
  src/run_polygon_edge_tests_given_obstacle_map.cpp
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
target_link_libraries(polyanya_wrapper PRIVATE visibility OpenMP::OpenMP_CXX)

pybind11_add_module(vgraph_planner
  src/bind_vgraph_planner.cpp
  src/plan_on_vgraph.cpp
)
target_link_libraries(vgraph_planner PRIVATE visibility OpenMP::OpenMP_CXX)
