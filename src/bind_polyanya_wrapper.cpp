#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include "polyanya_wrapper/polyanya_wrapper.h"
#include "polyanya_wrapper/run_polygon_edge_tests_given_obstacle_map.h"

namespace py = pybind11;

PYBIND11_MODULE(polyanya_wrapper, m) {
  py::class_<PolyanyaWrapper>(m, "PolyanyaWrapper")
    .def(py::init<const Ref<const Matrix<bool, Dynamic, Dynamic, RowMajor>>&, const std::string&, const std::string&>())
    .def("shortest_path", &PolyanyaWrapper::shortest_path)
    ;

  m.def("run_polygon_edge_tests_given_obstacle_map", &run_polygon_edge_tests_given_obstacle_map);
  m.def("run_polygon_edge_tests_given_obstacle_map_with_vgraph_option", &run_polygon_edge_tests_given_obstacle_map_with_vgraph_option);
}
