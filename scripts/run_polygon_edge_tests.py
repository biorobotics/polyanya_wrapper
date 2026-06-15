import numpy as np
import sys
sys.path.append('../build/')
import os
from polyanya_wrapper import run_polygon_edge_tests_given_obstacle_map, PolyanyaWrapper
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from png_to_boxes import im_to_rects, rects_to_polytopes, get_shared_rect_boundaries, get_connected_rect_component, merge_rects_into_polygons
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

original_epsilon = False

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

  do_plot = False
  # do_plot = obstacle_map_idx == 78
  if do_plot:
    test_idx = 4
    start = results[test_idx, :2]
    goal = results[test_idx, 2:4]

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

    ax.scatter([start[0]], [start[1]], color='red', label='start position')
    ax.scatter([goal[0]], [goal[1]], color='aqua', label='goal position')

    ax.plot(path[:, 0], path[:, 1], color='blue', linewidth=4, solid_capstyle='butt')

    plt.xlim(0, occupancy.shape[0])
    plt.ylim(0, occupancy.shape[1])
    plt.legend(fontsize=15)
    ax.set_aspect('equal')
    plt.show()

do_save = False
if do_save:
  if original_epsilon:
    np.save('results_original_epsilon.npy', overall_results)
  else:
    np.save('results_new_epsilon.npy', overall_results)
