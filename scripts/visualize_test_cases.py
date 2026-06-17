import numpy as np
import pickle
import sys
sys.path.append('../build/')

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from png_to_boxes import im_to_rects, rects_to_polytopes, get_shared_rect_boundaries, get_connected_rect_component, merge_rects_into_polygons
from construct_standard_vg import construct_vg
matplotlib.use('TkAgg')
matplotlib.rcParams.update({'font.size': 45})

num_obstacle_maps = 100
num_tests_per_obstacle_map = 100

for test_idx in range(num_tests_per_obstacle_map):
  for obstacle_map_idx in range(num_obstacle_maps):
    print('Obstacle map %d, test %d' %(obstacle_map_idx, test_idx))
    instance_path = '../test_cases/obstacle_map' + str(obstacle_map_idx)
    occupancy = np.load(instance_path + '/occupancy_grid.npy')
    start_goal_pairs = np.load(instance_path + '/start_goal_pairs.npy')
    path_lengths = np.load(instance_path + '/path_lengths.npy')
    if np.isinf(path_lengths[test_idx]):
      continue
    with open(instance_path + '/vgraph_paths.pkl', 'rb') as f:
      vgraph_paths = pickle.load(f)

    start = start_goal_pairs[test_idx, :2]
    goal = start_goal_pairs[test_idx, 2:]
    path = vgraph_paths[test_idx]
    diff = abs(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)) - path_lengths[test_idx])
    assert diff < 1e-8, diff

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
    plt.legend(fontsize=15)
    ax.set_aspect('equal')
    plt.show()
    plt.close()
