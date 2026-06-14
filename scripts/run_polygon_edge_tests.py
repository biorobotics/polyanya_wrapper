import numpy as np
import sys
sys.path.append('../build/')
import os
from polyanya_wrapper import run_polygon_edge_tests_given_obstacle_map

repo_path = '../'

num_obstacle_maps = 100
num_tests_per_obstacle_map = 100
num_grid_cells_per_axis = 30
obstacle_density = 0.2

map_save_path = repo_path + '/data/tmp/'

home = os.path.expanduser("~")
polyanya_path = home + '/polyanya/anyangle/polyanya/'

random_seed = 0
rng = np.random.default_rng(np.random.SeedSequence(12345).spawn(random_seed + 1)[-1])

dim_q = 2
overall_results = np.zeros((0, 2*dim_q + 1))

use_epsilon = False

for obstacle_map_idx in range(num_obstacle_maps):
  print('Running tests for obstacle map %d' %(obstacle_map_idx))
  occupancy = np.zeros((num_grid_cells_per_axis, num_grid_cells_per_axis), dtype=bool)
  available_cell_indices = np.where(np.logical_not(occupancy))
  indices_idx = rng.choice(len(available_cell_indices[0]), size=int(obstacle_density*len(available_cell_indices[0])), replace=False)
  occupied_cell_indices = (available_cell_indices[0][indices_idx], available_cell_indices[1][indices_idx])
  for cell_x, cell_y in zip(*occupied_cell_indices):
    occupancy[cell_x, cell_y] = True

  results = run_polygon_edge_tests_given_obstacle_map(occupancy.transpose(), polyanya_path, map_save_path, num_tests_per_obstacle_map)
  overall_results = np.concatenate((overall_results, results))

if use_epsilon:
  np.save('results_with_epsilon.npy', overall_results)
else:
  np.save('results_without_epsilon.npy', overall_results)
