import numpy as np
import sys
sys.path.append('../build/')
import os
from polyanya_wrapper2 import PolyanyaWrapper2
from vgraph_planner import plan_on_vgraph
from png_to_boxes import im_to_rects, rects_to_polytopes, get_shared_rect_boundaries, get_connected_rect_component, merge_rects_into_polygons
from construct_standard_vg import construct_vg

repo_path = '../'

num_obstacle_maps = 100
num_tests_per_obstacle_map = 100
num_grid_cells_per_axis = 30
obstacle_density = 0.2

map_save_path = repo_path + '/data/tmp/'

home = os.path.expanduser("~")
polyanya_path = home + '/polyanya/anyangle/polyanya/'

dim_q = 2
overall_results = np.zeros((num_obstacle_maps*num_tests_per_obstacle_map, 2*dim_q + 1))

use_vgraph = False
result_idx = 0

for obstacle_map_idx in range(num_obstacle_maps):
  print('Running tests for obstacle map %d' %(obstacle_map_idx))

  instance_path = '../test_cases/obstacle_map' + str(obstacle_map_idx)
  occupancy = np.load(instance_path + '/occupancy_grid.npy')
  start_goal_pairs = np.load(instance_path + '/start_goal_pairs.npy')

  for test_idx in range(num_tests_per_obstacle_map):
    print('Obstacle map %d, test %d' %(obstacle_map_idx, test_idx))
    start = start_goal_pairs[test_idx, :2]
    goal = start_goal_pairs[test_idx, 2:]
    overall_results[result_idx, :4] = start_goal_pairs[test_idx]

    polyanya_wrapper = PolyanyaWrapper2(occupancy.transpose(), \
                                        polyanya_path, \
                                        map_save_path)
    try:
      path = polyanya_wrapper.shortest_path(start, goal)
      if len(path) == 0:
        path_length = np.inf
      else:
        path_length = np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1))
      overall_results[result_idx, 4] = path_length
    except:
      overall_results[result_idx, 4] = np.nan

    result_idx += 1

do_save = True
if do_save:
  print('Saving results')
  if use_vgraph:
    np.save('results_vgraph.npy', overall_results)
  else:
    np.save('results_polyanya2.npy', overall_results)
