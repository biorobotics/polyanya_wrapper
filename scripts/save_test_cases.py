import numpy as np
from png_to_boxes import im_to_rects, rects_to_polytopes, get_shared_rect_boundaries, get_connected_rect_component, merge_rects_into_polygons
from geometry_utils import *
import os
import pickle
from construct_standard_vg import construct_vg
from vgraph_planner import plan_on_vgraph

results_vgraph = np.load('results_vgraph.npy')

num_obstacle_maps = 100
num_grid_cells_per_axis = 30
num_tests_per_obstacle_map = 100
obstacle_density = 0.2
occupancy_grids = []
random_seed = 0
rng = np.random.default_rng(np.random.SeedSequence(12345).spawn(random_seed + 1)[-1])
for obstacle_map_idx in range(num_obstacle_maps):
  save_path = '../test_cases/obstacle_map' + str(obstacle_map_idx)
  if not os.path.exists(save_path):
    os.makedirs(save_path)
  occupancy = np.zeros((num_grid_cells_per_axis, num_grid_cells_per_axis), dtype=bool)
  available_cell_indices = np.where(np.logical_not(occupancy))
  indices_idx = rng.choice(len(available_cell_indices[0]), size=int(obstacle_density*len(available_cell_indices[0])), replace=False)

  occupied_cell_indices = (available_cell_indices[0][indices_idx], available_cell_indices[1][indices_idx])
  for cell_x, cell_y in zip(*occupied_cell_indices):
    occupancy[cell_x, cell_y] = True

  np.save(save_path + '/occupancy_grid.npy', occupancy)
  np.save(save_path + '/start_goal_pairs.npy', results_vgraph[num_tests_per_obstacle_map*obstacle_map_idx:num_tests_per_obstacle_map*(obstacle_map_idx + 1), :4])
  np.save(save_path + '/path_lengths.npy', results_vgraph[num_tests_per_obstacle_map*obstacle_map_idx:num_tests_per_obstacle_map*(obstacle_map_idx + 1), 4])

  vgraph_paths = []

  obstacles_info = (occupancy.astype(int)*255, np.zeros(2), occupancy.shape)
  vgraph_indptr, vgraph_indices, vgraph_data, obstacle_vertices, visibility_wrapper = construct_vg(obstacles_info, np.inf)
  for start_goal_pair in results_vgraph[num_tests_per_obstacle_map*obstacle_map_idx:num_tests_per_obstacle_map*(obstacle_map_idx + 1), :4]:
    start = start_goal_pair[:2]
    goal = start_goal_pair[2:]
    path = plan_on_vgraph(vgraph_indptr, vgraph_indices, vgraph_data, obstacle_vertices, visibility_wrapper, start, goal)

    vgraph_paths.append(np.array(path))

  with open(save_path + '/vgraph_paths.pkl', 'wb') as f:
    pickle.dump(vgraph_paths, f)
