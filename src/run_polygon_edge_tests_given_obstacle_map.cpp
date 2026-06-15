#include "polyanya_wrapper/run_polygon_edge_tests_given_obstacle_map.h"
#include <random>
#include <iomanip>

MatrixXd run_polygon_edge_tests_given_obstacle_map(const Ref<const Matrix<bool, Dynamic, Dynamic, RowMajor>> &occupancy, const std::string &polyanya_path, const std::string &map_save_path, int num_tests) {
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

  return ret;
}
