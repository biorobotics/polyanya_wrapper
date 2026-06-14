#pragma once
#include "polyanya_wrapper/polyanya_wrapper.h"

// Returns matrix where columns from left to right are, x and y coordinates of start point, x and y coordinates of goal point, and path length. Number of rows is num_tests
MatrixXd run_polygon_edge_tests_given_obstacle_map(const Ref<const Matrix<bool, Dynamic, Dynamic, RowMajor>> &occupancy, const std::string &polyanya_path, const std::string &map_save_path, int num_tests);
