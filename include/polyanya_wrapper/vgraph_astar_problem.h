#pragma once
#include "polyanya_wrapper/astar_problem.h"
#include <memory>
#include "polyanya_wrapper/arastar.h"

typedef Matrix<long, Dynamic, 1> VectorXl;

class VGraphAStarProblem : public AStarProblem {
  public:
    VGraphAStarProblem(const std::vector<std::vector<std::pair<int, double>>> &adj_list,
                       const VectorXd &h_vals_per_obstacle_vertex,
                       int start_node_idx,
                       int goal_node_idx) : adj_list(adj_list), 
                                            h_vals_per_obstacle_vertex(h_vals_per_obstacle_vertex),
                                            start_node_idx(start_node_idx),
                                            goal_node_idx(goal_node_idx) {
    }

    virtual void generate_successors(std::vector<AStarCell> &succ, std::vector<double> &transition_costs, const AStarNodePtr &node) const override {
      succ.clear();
      transition_costs.clear();

      int node_idx = node->get_cell()(0);
      if (node_idx == goal_node_idx) {
        return;
      }

      VectorXi next_cell(1);

      for (const std::pair<int, double> &neighbor : adj_list[node_idx]) {
        if (neighbor.first != goal_node_idx && neighbor.first >= h_vals_per_obstacle_vertex.size()) {
          // Don't go to non-obstacle vertices that aren't the goal
          continue;
        }
        next_cell(0) = neighbor.first;
        succ.push_back(next_cell);
        transition_costs.push_back(neighbor.second);
      }
    }

    virtual bool is_goal(const AStarNodePtr &node) const override {
      return node->get_cell()(0) == goal_node_idx;
    }

    virtual double heuristic(const AStarCell &cell) const override {
      int node_idx = cell(0);
      if (node_idx == start_node_idx || node_idx == goal_node_idx) {
        return 0;
      }

      if (node_idx >= h_vals_per_obstacle_vertex.size()) {
        throw std::runtime_error("We should not be computing the heuristic for a non-obstacle vertex that is not the start or goal");
      } else {
        return h_vals_per_obstacle_vertex(node_idx);
      }
    }

  private:
    const std::vector<std::vector<std::pair<int, double>>> &adj_list;
    const VectorXd &h_vals_per_obstacle_vertex;
    int start_node_idx;
    int goal_node_idx;
};

double vgraph_astar(std::vector<int> &node_seq,
                    bool populate_node_seq,
                    const std::vector<std::vector<std::pair<int, double>>> &adj_list,
                    const VectorXd &h_vals_per_obstacle_vertex,
                    int start_node_idx,
                    int goal_node_idx) {
  std::shared_ptr<VGraphAStarProblem> problem = std::make_shared<VGraphAStarProblem>(adj_list,
                                                                                     h_vals_per_obstacle_vertex,
                                                                                     start_node_idx,
                                                                                     goal_node_idx);

  AStarCell start_cell = VectorXi(1);
  start_cell(0) = start_node_idx;
  std::shared_ptr<ARAStarData> astar_data = std::make_shared<ARAStarData>(problem, start_cell);
  AStarPath path;
  if (arastar(path, astar_data, 1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity())) {
    if (populate_node_seq) {
      node_seq.resize(path.size());
      for (int i = 0; i < path.size(); ++i) {
        node_seq[i] = path[i](0);
      }
    }
    return astar_data->get_f_goal();
  } else {
    return std::numeric_limits<double>::infinity();
  }
}
