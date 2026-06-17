import numpy as np
from png_to_boxes import im_to_rects, rects_to_polytopes, get_shared_rect_boundaries, get_connected_rect_component, merge_rects_into_polygons
from geometry_utils import *

num_obstacle_maps = 100
num_grid_cells_per_axis = 30
obstacle_density = 0.2
obstacle_rects_per_map = []
random_seed = 0
rng = np.random.default_rng(np.random.SeedSequence(12345).spawn(random_seed + 1)[-1])
for obstacle_map_idx in range(num_obstacle_maps):
  occupancy = np.zeros((num_grid_cells_per_axis, num_grid_cells_per_axis), dtype=bool)
  available_cell_indices = np.where(np.logical_not(occupancy))
  indices_idx = rng.choice(len(available_cell_indices[0]), size=int(obstacle_density*len(available_cell_indices[0])), replace=False)

  occupied_cell_indices = (available_cell_indices[0][indices_idx], available_cell_indices[1][indices_idx])
  for cell_x, cell_y in zip(*occupied_cell_indices):
    occupancy[cell_x, cell_y] = True

  num_cells_x = occupancy.shape[0]
  num_cells_y = occupancy.shape[1]
  obstacle_rects = im_to_rects(np.copy((occupancy.astype(int)*255).reshape(num_cells_x, num_cells_y, 1)))
  obstacle_rects_per_map.append(np.array(obstacle_rects).astype(float))

results_vgraph = np.load('results_vgraph.npy')
results_polyanya = np.load('results_original_epsilon.npy')

max_path_length_ratio = 0.
global_test_idx_with_min_path_length_ratio = None
instance_with_min_path_length_ratio = None
min_path_length_ratio = np.inf
num_instances_where_vgraph_found_path_and_polyanya_didnt = 0
instances_with_error = []
instances_with_large_path_length_difference_from_vgraph_path = []
instances_with_no_path_where_vgraph_path_was_found = []
for global_test_idx, (result_vgraph, result_polyanya) in enumerate(zip(results_vgraph, results_polyanya)):
  obstacle_map_idx = global_test_idx//100
  test_idx = global_test_idx%100
  assert(np.all(result_vgraph[:4] == result_polyanya[:4]))
  if np.isnan(result_polyanya[4]):
    instances_with_error.append((obstacle_map_idx, test_idx))
    continue

  if np.isinf(result_polyanya[4]):
    if np.isfinite(result_vgraph[4]):
      num_instances_where_vgraph_found_path_and_polyanya_didnt += 1
      instances_with_no_path_where_vgraph_path_was_found.append((obstacle_map_idx, test_idx))
    continue

  if np.isinf(result_vgraph[4]):
    close_to_obstacle_edge = False
    for rect in obstacle_rects_per_map[obstacle_map_idx]:
      minx = rect[0, 0]
      miny = rect[0, 1]
      maxx = rect[1, 0]
      maxy = rect[1, 1]
      edge1 = ((minx, miny), (minx, maxy))
      edge2 = ((minx, maxy), (maxx, maxy))
      edge3 = ((maxx, maxy), (maxx, miny))
      edge4 = ((maxx, miny), (minx, miny))
      start = result_vgraph[:2]
      d1 = dist_point_to_line_segment(*edge1[0], *edge1[1], *start)
      d2 = dist_point_to_line_segment(*edge2[0], *edge2[1], *start)
      d3 = dist_point_to_line_segment(*edge3[0], *edge3[1], *start)
      d4 = dist_point_to_line_segment(*edge4[0], *edge4[1], *start)
      d_start = min(d1, d2, d3, d4)

      if d_start <= 1e-7:
        close_to_obstacle_edge = True
        break

      '''
      goal = result_vgraph[2:4]
      d1 = dist_point_to_line_segment(*edge1[0], *edge1[1], *goal)
      d2 = dist_point_to_line_segment(*edge2[0], *edge2[1], *goal)
      d3 = dist_point_to_line_segment(*edge3[0], *edge3[1], *goal)
      d4 = dist_point_to_line_segment(*edge4[0], *edge4[1], *goal)
      d_goal = min(d1, d2, d3, d4)
      if min(d_start, d_goal) <= 1e-7:
        close_to_obstacle_edge = True
        break
      '''

    assert(close_to_obstacle_edge)
    continue

  path_length_ratio = result_vgraph[4]/result_polyanya[4]
  if path_length_ratio < min_path_length_ratio:
    global_test_idx_with_min_path_length_ratio = global_test_idx
    instance_with_min_path_length_ratio = (obstacle_map_idx, test_idx)
    min_path_length_ratio = path_length_ratio

  max_path_length_ratio = max(max_path_length_ratio, path_length_ratio)

  if result_vgraph[4] - result_polyanya[4] > 1e-4:
    assert(False)

  if result_vgraph[4] - result_polyanya[4] < -1e-4:
    instances_with_large_path_length_difference_from_vgraph_path.append((obstacle_map_idx, test_idx))

print('A* on vgraph found path in %d instances where polyanya terminated without error but did not find path' %(num_instances_where_vgraph_found_path_and_polyanya_didnt))
print('Vgraph path is at most %f times shorter than polyanya path' %(1/min_path_length_ratio))
print('Vgraph path is at most 1 + %E times longer than polyanya path' %(max_path_length_ratio - 1))
print('Obstacle map of min ratio of vgraph path length to polyanya path length: %d' %(instance_with_min_path_length_ratio[0]))
print('Test index of min ratio of vgraph path length to polyanya path length: %d' %(instance_with_min_path_length_ratio[1]))
print('Path length from vgraph is %f, and from polyanya is %f' %(results_vgraph[global_test_idx_with_min_path_length_ratio, 4], results_polyanya[global_test_idx_with_min_path_length_ratio, 4]))
print('Ratio of vgraph path length to polyanya path length is %f' %(results_vgraph[global_test_idx_with_min_path_length_ratio, 4]/results_polyanya[global_test_idx_with_min_path_length_ratio, 4]))

instances_with_large_path_length_difference_from_vgraph_path = np.array(instances_with_large_path_length_difference_from_vgraph_path)
np.save('instances_with_large_path_length_difference_from_vgraph_path.npy', instances_with_large_path_length_difference_from_vgraph_path)

instances_with_no_path_where_vgraph_path_was_found = np.array(instances_with_no_path_where_vgraph_path_was_found)
np.save('instances_with_no_path_where_vgraph_path_was_found.npy', instances_with_no_path_where_vgraph_path_was_found)

instances_with_error = np.array(instances_with_error)
np.save('instances_with_error.npy', instances_with_error)
