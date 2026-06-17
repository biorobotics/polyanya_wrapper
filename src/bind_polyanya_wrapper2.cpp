#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include "polyanya_wrapper/polyanya_wrapper2.h"

namespace py = pybind11;

PYBIND11_MODULE(polyanya_wrapper2, m) {
  py::class_<PolyanyaWrapper2>(m, "PolyanyaWrapper2")
    .def(py::init<const Ref<const Matrix<bool, Dynamic, Dynamic, RowMajor>>&, const std::string&, const std::string&>())
    .def("shortest_path", &PolyanyaWrapper2::shortest_path)
    ;
}
