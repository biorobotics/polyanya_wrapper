#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include "polyanya_wrapper/visibility_wrapper.h"

namespace py = pybind11;

PYBIND11_MODULE(visibility_bindings, m) {
  py::class_<VisibilityWrapper, std::shared_ptr<VisibilityWrapper>>(m, "VisibilityWrapper")
    .def(py::init<Ref<Matrix<double, Dynamic, Dynamic, RowMajor>>, Ref<Matrix<double, Dynamic, Dynamic, RowMajor>>, Ref<VectorXl>>())
    .def("visibility_polygon", &VisibilityWrapper::visibility_polygon)
    .def("visibility_polygon_interior", &VisibilityWrapper::visibility_polygon_interior)
    ;
}
