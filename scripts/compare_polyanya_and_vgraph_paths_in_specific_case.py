import numpy as np
import sys
sys.path.append('../build/')
import os
from polyanya_wrapper import run_polygon_edge_tests_given_obstacle_map, run_polygon_edge_tests_given_obstacle_map_with_vgraph_option, PolyanyaWrapper
from vgraph_planner import plan_on_vgraph
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from png_to_boxes import im_to_rects, rects_to_polytopes, get_shared_rect_boundaries, get_connected_rect_component, merge_rects_into_polygons
from construct_standard_vg import construct_vg
matplotlib.use('TkAgg')

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

original_epsilon = True

use_vgraph = False

# Polyanya doesn't find path
obstacle_map_idx_for_plot = 0
test_idx_for_plot = 46

# Polyanya gives suboptimal path
# obstacle_map_idx_for_plot = 27
# test_idx_for_plot = 57

for obstacle_map_idx in range(num_obstacle_maps):
  # We have to generate all the obstacle maps up to obstacle_map_idx, the way this code works
  occupancy = np.zeros((num_grid_cells_per_axis, num_grid_cells_per_axis), dtype=bool)
  available_cell_indices = np.where(np.logical_not(occupancy))
  indices_idx = rng.choice(len(available_cell_indices[0]), size=int(obstacle_density*len(available_cell_indices[0])), replace=False)

  if obstacle_map_idx != obstacle_map_idx_for_plot:
    continue

  occupied_cell_indices = (available_cell_indices[0][indices_idx], available_cell_indices[1][indices_idx])
  for cell_x, cell_y in zip(*occupied_cell_indices):
    occupancy[cell_x, cell_y] = True

  if use_vgraph:
    obstacles_info = (occupancy.astype(int)*255, np.zeros(2), occupancy.shape)
    vgraph_indptr, vgraph_indices, vgraph_data, obstacle_vertices, visibility_wrapper = construct_vg(obstacles_info, np.inf)
    results = run_polygon_edge_tests_given_obstacle_map_with_vgraph_option(occupancy.transpose(), polyanya_path, map_save_path, num_tests_per_obstacle_map, vgraph_indptr, vgraph_indices, vgraph_data, obstacle_vertices, visibility_wrapper)
  else:
    results = run_polygon_edge_tests_given_obstacle_map(occupancy.transpose(), polyanya_path, map_save_path, num_tests_per_obstacle_map)
  overall_results = np.concatenate((overall_results, results))

  start = results[test_idx_for_plot, :2]
  goal = results[test_idx_for_plot, 2:4]

  if use_vgraph:
    path = plan_on_vgraph(vgraph_indptr, vgraph_indices, vgraph_data, obstacle_vertices, visibility_wrapper, start, goal)
  else:
    polyanya_wrapper = PolyanyaWrapper(occupancy.transpose(), \
                                       polyanya_path, \
                                       map_save_path)
    print('Planning path')
    path = polyanya_wrapper.shortest_path(start, goal)

  matplotlib.rcParams.update({'font.size': 45})
  prop_cycle = plt.rcParams['axes.prop_cycle']
  prop_cycle_colors = prop_cycle.by_key()['color']

  plt.figure(figsize=(15,15))
  ax = plt.gca()

  obstacle_rects = im_to_rects(np.copy((occupancy.astype(int)*255).reshape(*occupancy.shape, 1)))
  obstacle_color = 'chocolate'
  for rect_idx in range(len(obstacle_rects)):
    rect = obstacle_rects[rect_idx]
    xy = (rect[0][0], rect[0][1])
    width = rect[1][0] - xy[0]
    height = rect[1][1] - xy[1]
    ax.add_patch(patches.Rectangle(xy, width, height, facecolor=obstacle_color))

  ax.scatter([start[0]], [start[1]], color='red', label='start position', s=200)
  ax.scatter([goal[0]], [goal[1]], color='aqua', label='goal position', s=200)

  ax.plot(path[:, 0], path[:, 1], color='blue', linewidth=4, solid_capstyle='butt')

  plt.xlim(0, occupancy.shape[0])
  plt.ylim(0, occupancy.shape[1])
  # plt.legend(fontsize=15)
  ax.set_aspect('equal')
  if use_vgraph:
    plt.savefig('../plots/vgraph_path.png')
  else:
    plt.savefig('../plots/polyanya_path.png')
  plt.show()
  quit()
