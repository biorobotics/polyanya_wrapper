#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl_bind.h>
#include "polyanya_wrapper/plan_on_vgraph.h"

namespace py = pybind11;

PYBIND11_MODULE(vgraph_planner, m) {
  m.def("plan_on_vgraph", &plan_on_vgraph);
}
