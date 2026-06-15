#pragma once
#include "polyanya_wrapper/visibility_wrapper.h"

MatrixXd plan_on_vgraph(const Ref<const VectorXl> &vgraph_indptr, const Ref<const VectorXl> &vgraph_indices, const Ref<const VectorXd> &vgraph_data, const Ref<const MatrixXd> &obstacle_vertices, std::shared_ptr<VisibilityWrapper> visibility_wrapper, const Ref<const Vector2d> &start, const Ref<const Vector2d> &goal);
