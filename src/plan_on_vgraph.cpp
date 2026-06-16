#include "polyanya_wrapper/plan_on_vgraph.h"
#include "polyanya_wrapper/vgraph_astar_problem.h"

class double_pair_hash {
  public:
    std::size_t operator()(const std::pair<double, double> &pair) const {
        return std::hash<double>()(pair.first) ^ std::hash<double>()(pair.second);
    }
};

bool is_point_in_polygon(const Ref<const Vector2d> &point, const Ref<const MatrixXd> &vpoly) {
  std::vector<Vector2d> vertices;
  for (int i = 0; i < vpoly.rows(); ++i) {
      vertices.push_back(vpoly.row(i).head<2>());
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

MatrixXd plan_on_vgraph(const Ref<const VectorXl> &vgraph_indptr, const Ref<const VectorXl> &vgraph_indices, const Ref<const VectorXd> &vgraph_data, const Ref<const MatrixXd> &obstacle_vertices, std::shared_ptr<VisibilityWrapper> visibility_wrapper, const Ref<const Vector2d> &start, const Ref<const Vector2d> &goal) {
  if (visibility_wrapper->point_in_obstacle(start) || visibility_wrapper->point_in_obstacle(goal)) {
    return MatrixXd::Zero(0, 2);
  }

  MatrixXd vpoly_start = visibility_wrapper->visibility_polygon_interior(start);

  if (is_point_in_polygon(goal, vpoly_start)) {
    MatrixXd ret(2, 2);
    ret.row(0) = start.transpose();
    ret.row(1) = goal.transpose();
    return ret;
  }

  std::vector<std::vector<std::pair<int, double>>> vgraph_adj_list(vgraph_indptr.size() - 1);
  std::unordered_map<std::pair<double, double>, int, double_pair_hash> vertex_loc_to_idx_map(vgraph_indptr.size() - 1);
  for (int node_idx = 0; node_idx < vgraph_indptr.size() - 1; ++node_idx) {
    for (int col_idx = vgraph_indptr(node_idx); col_idx < vgraph_indptr(node_idx + 1); ++col_idx) {
      vgraph_adj_list[node_idx].push_back(std::pair<int, double>(vgraph_indices(col_idx), vgraph_data(col_idx)));
    }
  }

  for (int vertex_idx = 0; vertex_idx < obstacle_vertices.rows(); ++vertex_idx) {
    vertex_loc_to_idx_map[std::pair<double, double>(obstacle_vertices(vertex_idx, 0), obstacle_vertices(vertex_idx, 1))] = vertex_idx;
  }

  std::vector<int> vgraph_node_seq;

  MatrixXd vpoly_goal = visibility_wrapper->visibility_polygon_interior(goal);
  int start_node_idx = vgraph_adj_list.size();
  vgraph_adj_list.push_back(std::vector<std::pair<int, double>>());
  int goal_node_idx = vgraph_adj_list.size();
  vgraph_adj_list.push_back(std::vector<std::pair<int, double>>());
  for (int row = 0; row < vpoly_start.rows(); ++row) {
    auto it = vertex_loc_to_idx_map.find(std::pair<double, double>(vpoly_start(row, 0), vpoly_start(row, 1)));
    if (it != vertex_loc_to_idx_map.end()) {
      int vertex_idx = it->second;
      double dist = (obstacle_vertices.row(vertex_idx).transpose() - start).norm();
      vgraph_adj_list[start_node_idx].push_back(std::pair<int, double>(vertex_idx, dist));
      vgraph_adj_list[vertex_idx].push_back(std::pair<int, double>(start_node_idx, dist));
    }
  }

  for (int row = 0; row < vpoly_goal.rows(); ++row) {
    auto it = vertex_loc_to_idx_map.find(std::pair<double, double>(vpoly_goal(row, 0), vpoly_goal(row, 1)));
    if (it != vertex_loc_to_idx_map.end()) {
      int vertex_idx = it->second;
      double dist = (obstacle_vertices.row(vertex_idx).transpose() - goal).norm();
      vgraph_adj_list[goal_node_idx].push_back(std::pair<int, double>(vertex_idx, dist));
      vgraph_adj_list[vertex_idx].push_back(std::pair<int, double>(goal_node_idx, dist));
    }
  }

  VectorXd h_vals_per_obstacle_vertex(obstacle_vertices.rows());
  for (int vertex_idx = 0; vertex_idx < obstacle_vertices.rows(); ++vertex_idx) {
    h_vals_per_obstacle_vertex(vertex_idx) = (goal - obstacle_vertices.row(vertex_idx).transpose()).norm();
  }
  double dist = vgraph_astar(vgraph_node_seq, true, vgraph_adj_list, h_vals_per_obstacle_vertex, start_node_idx, goal_node_idx);
  if (std::isinf(dist)) {
    return MatrixXd::Zero(0, 2);
  }
  MatrixXd ret(vgraph_node_seq.size(), 2);
  ret.row(0) = start.transpose();
  ret.row(vgraph_node_seq.size() - 1) = goal.transpose();
  for (int seq_idx = 1; seq_idx < vgraph_node_seq.size() - 1; ++seq_idx) {
    int node_idx = vgraph_node_seq[seq_idx];
    ret.row(seq_idx) = obstacle_vertices.row(node_idx);
  }

  return ret;
}
