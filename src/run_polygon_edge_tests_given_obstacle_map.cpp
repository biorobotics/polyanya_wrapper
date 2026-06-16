#include "polyanya_wrapper/run_polygon_edge_tests_given_obstacle_map.h"
#include "polyanya_wrapper/vgraph_astar_problem.h"
#include <random>
#include <iomanip>

class double_pair_hash {
  public:
    std::size_t operator()(const std::pair<double, double> &pair) const {
        return std::hash<double>()(pair.first) ^ std::hash<double>()(pair.second);
    }
};

MatrixXd run_polygon_edge_tests_given_obstacle_map(const Ref<const Matrix<bool, Dynamic, Dynamic, RowMajor>> &occupancy, const std::string &polyanya_path, const std::string &map_save_path, int num_tests) {
  return run_polygon_edge_tests_given_obstacle_map_with_vgraph_option(occupancy, polyanya_path, map_save_path, num_tests, VectorXl::Zero(0), VectorXl::Zero(0), VectorXd::Zero(0), MatrixXd::Zero(0, 2), nullptr);
}

MatrixXd run_polygon_edge_tests_given_obstacle_map_with_vgraph_option(const Ref<const Matrix<bool, Dynamic, Dynamic, RowMajor>> &occupancy, const std::string &polyanya_path, const std::string &map_save_path, int num_tests, const Ref<const VectorXl> &vgraph_indptr, const Ref<const VectorXl> &vgraph_indices, const Ref<const VectorXd> &vgraph_data, const Ref<const MatrixXd> &obstacle_vertices, std::shared_ptr<VisibilityWrapper> visibility_wrapper) {
  std::vector<std::vector<std::pair<int, double>>> vgraph_adj_list;
  std::unordered_map<std::pair<double, double>, int, double_pair_hash> vertex_loc_to_idx_map;

  bool use_vgraph = visibility_wrapper != nullptr;
  if (use_vgraph) {
    vgraph_adj_list.resize(vgraph_indptr.size() - 1);
    for (int node_idx = 0; node_idx < vgraph_indptr.size() - 1; ++node_idx) {
      for (int col_idx = vgraph_indptr(node_idx); col_idx < vgraph_indptr(node_idx + 1); ++col_idx) {
        vgraph_adj_list[node_idx].push_back(std::pair<int, double>(vgraph_indices(col_idx), vgraph_data(col_idx)));
      }
    }

    for (int vertex_idx = 0; vertex_idx < obstacle_vertices.rows(); ++vertex_idx) {
      vertex_loc_to_idx_map[std::pair<double, double>(obstacle_vertices(vertex_idx, 0), obstacle_vertices(vertex_idx, 1))] = vertex_idx;
    }
  }
  std::vector<int> dummy_vgraph_node_seq;

  PolyanyaWrapper polyanya_wrapper(occupancy, polyanya_path, map_save_path);
  std::shared_ptr<polyanya::Mesh> mesh = polyanya_wrapper.mesh;
  unsigned int random_seed = 4223100027;
  std::mt19937 rng(random_seed);
  std::uniform_int_distribution<int> polygon_distribution(0, mesh->mesh_polygons.size() - 1);
  std::uniform_real_distribution<double> alpha1_distribution(0, 1);
  std::uniform_real_distribution<double> alpha2_distribution(0, 1e-8);

  std::uniform_real_distribution<double> x_distribution(0, occupancy.cols());
  std::uniform_real_distribution<double> y_distribution(0, occupancy.rows());

  Matrix2d rot90;
  rot90(0, 0) = 0.;
  rot90(1, 0) = 1.;
  rot90(0, 1) = -1.;
  rot90(1, 1) = 0.;
  int dim_q = 2;
  MatrixXd ret(num_tests, 2*dim_q + 1);
  for (int test_idx = 0; test_idx < num_tests; ++test_idx) {
    // Pick random polygon
    int polygon_idx = polygon_distribution(rng);
    std::cout << test_idx << std::endl;

    int num_edges = mesh->mesh_polygons[polygon_idx].vertices.size();

    std::uniform_int_distribution<int> edge_distribution(0, num_edges - 1);

    // Pick random edge for start
    int edge_idx = edge_distribution(rng);
    int vertex1_idx = edge_idx;
    int vertex2_idx = edge_idx + 1;
    if (vertex2_idx >= num_edges) {
      vertex2_idx = 0;
    }

    // Pick random fraction to travel along edge
    double alpha1 = alpha1_distribution(rng);

    // Pick random fraction of 1e-8 to travel perpendicular to edge
    double alpha2 = alpha2_distribution(rng);

    Vector2d p1(mesh->mesh_vertices[mesh->mesh_polygons[polygon_idx].vertices[vertex1_idx]].p.x,
                mesh->mesh_vertices[mesh->mesh_polygons[polygon_idx].vertices[vertex1_idx]].p.y);

    Vector2d p2(mesh->mesh_vertices[mesh->mesh_polygons[polygon_idx].vertices[vertex2_idx]].p.x,
                mesh->mesh_vertices[mesh->mesh_polygons[polygon_idx].vertices[vertex2_idx]].p.y);

    Vector2d start = (1 - alpha1)*p1 + alpha1*p2;
    Vector2d perp = (rot90*(p2 - p1)).normalized();
    Vector2d tentative_start = start + alpha2*perp;
    Vector2i tentative_start_int = tentative_start.cast<int>();
    if (occupancy(tentative_start_int(1), tentative_start_int(0))) {
      start = start - alpha2*perp;
      if (occupancy((int)start(1), (int)start(0))) {
        throw std::runtime_error("Each perturbation of interpolated point causes collision.");
      }
    } else {
      start = tentative_start;
    }

    double x_goal = x_distribution(rng);
    double y_goal = y_distribution(rng);
    while (occupancy((int)y_goal, (int)x_goal)) {
      x_goal = x_distribution(rng);
      y_goal = y_distribution(rng);
    }
    Vector2d goal(x_goal, y_goal);

    ret(test_idx, 0) = start(0);
    ret(test_idx, 1) = start(1);
    ret(test_idx, 2) = goal(0);
    ret(test_idx, 3) = goal(1);

    if (use_vgraph) {
      if (visibility_wrapper->point_in_obstacle(start) || visibility_wrapper->point_in_obstacle(goal)) {
        ret(test_idx, 4) = std::numeric_limits<double>::infinity();
        continue;
      }
      MatrixXd vpoly_start = visibility_wrapper->visibility_polygon_interior(start);

      if (is_point_in_polygon(goal, vpoly_start)) {
        ret(test_idx, 4) = (goal - start).norm();
        continue;
      }

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
      int row = vpoly_start.rows() - 1;
      auto it = vertex_loc_to_idx_map.find(std::pair<double, double>(vpoly_start(row, 2), vpoly_start(row, 3)));
      if (it != vertex_loc_to_idx_map.end()) {
        int vertex_idx = it->second;
        double dist = (obstacle_vertices.row(vertex_idx).transpose() - start).norm();
        vgraph_adj_list[start_node_idx].push_back(std::pair<int, double>(vertex_idx, dist));
        vgraph_adj_list[vertex_idx].push_back(std::pair<int, double>(start_node_idx, dist));
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
      row = vpoly_goal.rows() - 1;
      it = vertex_loc_to_idx_map.find(std::pair<double, double>(vpoly_goal(row, 2), vpoly_goal(row, 3)));
      if (it != vertex_loc_to_idx_map.end()) {
        int vertex_idx = it->second;
        double dist = (obstacle_vertices.row(vertex_idx).transpose() - goal).norm();
        vgraph_adj_list[goal_node_idx].push_back(std::pair<int, double>(vertex_idx, dist));
        vgraph_adj_list[vertex_idx].push_back(std::pair<int, double>(goal_node_idx, dist));
      }

      VectorXd h_vals_per_obstacle_vertex(obstacle_vertices.rows());
      for (int vertex_idx = 0; vertex_idx < obstacle_vertices.rows(); ++vertex_idx) {
        h_vals_per_obstacle_vertex(vertex_idx) = (goal - obstacle_vertices.row(vertex_idx).transpose()).norm();
      }
      double dist = vgraph_astar(dummy_vgraph_node_seq, false, vgraph_adj_list, h_vals_per_obstacle_vertex, start_node_idx, goal_node_idx);
      ret(test_idx, 4) = dist;

      vgraph_adj_list.pop_back();
      vgraph_adj_list.pop_back();
      for (int row = 0; row < vpoly_start.rows(); ++row) {
        auto it = vertex_loc_to_idx_map.find(std::pair<double, double>(vpoly_start(row, 0), vpoly_start(row, 1)));
        if (it != vertex_loc_to_idx_map.end()) {
          int vertex_idx = it->second;
          vgraph_adj_list[vertex_idx].pop_back();
        }
      }
      row = vpoly_start.rows() - 1;
      it = vertex_loc_to_idx_map.find(std::pair<double, double>(vpoly_start(row, 2), vpoly_start(row, 3)));
      if (it != vertex_loc_to_idx_map.end()) {
        int vertex_idx = it->second;
        vgraph_adj_list[vertex_idx].pop_back();
      }

      for (int row = 0; row < vpoly_goal.rows(); ++row) {
        auto it = vertex_loc_to_idx_map.find(std::pair<double, double>(vpoly_goal(row, 0), vpoly_goal(row, 1)));
        if (it != vertex_loc_to_idx_map.end()) {
          int vertex_idx = it->second;
          vgraph_adj_list[vertex_idx].pop_back();
        }
      }
      row = vpoly_goal.rows() - 1;
      it = vertex_loc_to_idx_map.find(std::pair<double, double>(vpoly_goal(row, 2), vpoly_goal(row, 3)));
      if (it != vertex_loc_to_idx_map.end()) {
        int vertex_idx = it->second;
        vgraph_adj_list[vertex_idx].pop_back();
      }
    } else {
      MatrixXd shortest_path;
      try {
        shortest_path = polyanya_wrapper.shortest_path(start, goal);
      } catch (const std::runtime_error &error) {
        ret(test_idx, 4) = std::numeric_limits<double>::quiet_NaN();
        continue;
      }
      if (shortest_path.rows()) {
        double dist = 0.;
        for (int path_idx = 0; path_idx < shortest_path.rows() - 1; ++path_idx) {
          dist += (shortest_path.row(path_idx + 1) - shortest_path.row(path_idx)).norm();
        }
        ret(test_idx, 4) = dist;
      } else {
        ret(test_idx, 4) = std::numeric_limits<double>::infinity();
      }
    }
  }

  return ret;
}
