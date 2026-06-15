#include "polyanya_wrapper/visibility_wrapper.h"

bool is_point_in_polygon(const Ref<const Vector2d> &point, const Ref<const MatrixXd> &poly) {
  std::vector<Vector2d> vertices;
  for (int i = 0; i < poly.rows(); ++i) {
      vertices.push_back(poly.row(i).head<2>());
  }

  int num_crossings = 0;

  for (size_t i = 0; i < vertices.size(); ++i) {
        Vector2d v1 = vertices[i];
        Vector2d v2 = vertices[(i + 1) % vertices.size()];

        if ((v1(1) > point(1)) != (v2(1) > point(1))) {
            double x_intersection = v1(0) + (point(1) - v1(1)) * (v2(0) - v1(0)) / (v2(1) - v1(1));
            if (point(0) < x_intersection) {
                num_crossings++;
            }
        }
    }

  return (num_crossings % 2) == 1;
}

VisibilityWrapper::VisibilityWrapper(Ref<Matrix<double, Dynamic, Dynamic, RowMajor>> boundary_vertices, Ref<Matrix<double, Dynamic, Dynamic, RowMajor>> obstacle_vertices, Ref<VectorXl> obstacle_start_indices) : obstacle_vertices(obstacle_vertices), obstacle_start_indices(obstacle_start_indices) {
  std::vector<Segment_2> segments;
  for (int i = 0; i < boundary_vertices.rows() - 1; ++i) {
    double x1 = boundary_vertices(i, 0);
    double y1 = boundary_vertices(i, 1);
    double x2 = boundary_vertices(i + 1, 0);
    double y2 = boundary_vertices(i + 1, 1);
    Point_2 p1(x1, y1), p2(x2, y2);
    segments.push_back(Segment_2(p1, p2));
  }

  for (int i = 0; i < obstacle_start_indices.rows(); ++i) {
    int start_idx = obstacle_start_indices[i];
    int end_idx;
    if (i == obstacle_start_indices.rows() - 1) {
      end_idx = obstacle_vertices.rows();
    } else {
      end_idx = obstacle_start_indices[i + 1];
    }
    for (int j = start_idx; j < end_idx - 1; ++j) {
      vertex_to_obstacle_ptr.push_back(i);
      double x1 = obstacle_vertices(j, 0);
      double y1 = obstacle_vertices(j, 1);
      double x2 = obstacle_vertices(j + 1, 0);
      double y2 = obstacle_vertices(j + 1, 1);
      Point_2 p1(x1, y1), p2(x2, y2);
      segments.push_back(Segment_2(p1, p2));
    }
    vertex_to_obstacle_ptr.push_back(i);
  }
  CGAL::insert_non_intersecting_curves(env,segments.begin(),segments.end());
  tev = std::make_unique<TEV>(env);
}

MatrixXd VisibilityWrapper::visibility_polygon(long obstacle_vertex_idx) {
  double x1 = obstacle_vertices(obstacle_vertex_idx, 0);
  double y1 = obstacle_vertices(obstacle_vertex_idx, 1);

  int i = vertex_to_obstacle_ptr[obstacle_vertex_idx];
  int start_idx = obstacle_start_indices[i];
  int end_idx;

  if (obstacle_vertex_idx == start_idx) {
    std::cout << "Do not compute visibility from first vertex of obstacle polygon" << std::endl;
    exit(1);
  }

  double x2 = obstacle_vertices(obstacle_vertex_idx - 1, 0);
  double y2 = obstacle_vertices(obstacle_vertex_idx - 1, 1);

  Point_2 p1(x1, y1), p2(x2, y2);

  Halfedge_const_handle he = env.halfedges_begin();
  // while (he->source()->point() != p1 || he->target()->point() != p2) {
  while (he != env.halfedges_end() && (he->target()->point() != p1 || he->source()->point() != p2)) {
    he++;
  }
  if (he == env.halfedges_end()) {
    std::cout << "Query point does not correspond to any halfedge" << std::endl;
    exit(1);
  }

  Arrangement_2 output_arr;
  Face_handle fh = tev->compute_visibility(p1, he, output_arr);
  Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();
  std::vector<double> edge_coords;
  while (++curr != fh->outer_ccb()) {
    edge_coords.push_back(CGAL::to_double(curr->source()->point().x()));
    edge_coords.push_back(CGAL::to_double(curr->source()->point().y()));
    edge_coords.push_back(CGAL::to_double(curr->target()->point().x()));
    edge_coords.push_back(CGAL::to_double(curr->target()->point().y()));
  }
  double lastx = edge_coords[edge_coords.size() - 2];
  double lasty = edge_coords[edge_coords.size() - 1];
  edge_coords.push_back(lastx);
  edge_coords.push_back(lasty);
  edge_coords.push_back(edge_coords[0]);
  edge_coords.push_back(edge_coords[1]);

  return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(edge_coords.data(), edge_coords.size()/4, 4);
}

MatrixXd VisibilityWrapper::visibility_polygon_interior(const Ref<const Vector2d> &pt) {
  Point_2 query_pt(pt(0), pt(1));

  // From https://github.com/d-krupke/pyvispoly/blob/main/src/pyvispoly/_cgal_bindings.cpp
  auto face = env.unbounded_face();
  auto hole_it = face->holes_begin();
  assert(hole_it != face->holes_end());
  auto f = (*hole_it)->twin()->face();
  if (f->is_unbounded()) {
    throw std::runtime_error("Bad arrangement. Face should not be unbounded.");
  }
  auto interior_face = f;

  Arrangement_2 output_arr;
  Face_handle fh = tev->compute_visibility(query_pt, interior_face, output_arr);
  Arrangement_2::Ccb_halfedge_circulator curr = fh->outer_ccb();
  std::vector<double> edge_coords;
  while (++curr != fh->outer_ccb()) {
    edge_coords.push_back(CGAL::to_double(curr->source()->point().x()));
    edge_coords.push_back(CGAL::to_double(curr->source()->point().y()));
    edge_coords.push_back(CGAL::to_double(curr->target()->point().x()));
    edge_coords.push_back(CGAL::to_double(curr->target()->point().y()));
  }
  double lastx = edge_coords[edge_coords.size() - 2];
  double lasty = edge_coords[edge_coords.size() - 1];
  edge_coords.push_back(lastx);
  edge_coords.push_back(lasty);
  edge_coords.push_back(edge_coords[0]);
  edge_coords.push_back(edge_coords[1]);
  return Map<Matrix<double, Dynamic, Dynamic, RowMajor>>(edge_coords.data(), edge_coords.size()/4, 4);
}

bool VisibilityWrapper::point_in_obstacle(const Ref<const Vector2d> &pt) {
  for (int start_idx_idx = 0; start_idx_idx < obstacle_start_indices.size(); ++start_idx_idx) {
    int start_idx = obstacle_start_indices(start_idx_idx);
    int end_idx; // Exclusive
    if (start_idx_idx == obstacle_start_indices.size() - 1) {
      end_idx = obstacle_vertices.rows();
    } else {
      end_idx = obstacle_start_indices(start_idx_idx + 1);
    }
    if (end_idx - start_idx < 3) {
      throw std::runtime_error("Obstacle must have at least 3 vertices");
    }
    if (is_point_in_polygon(pt, obstacle_vertices.block(start_idx, 0, end_idx - start_idx, 2))) {
      return true;
    }
  }
  return false;
}
