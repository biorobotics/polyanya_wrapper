import numpy as np
import sys
sys.path.append('../build/')
import os
from polyanya_wrapper import PolyanyaWrapper

repo_path = '../'

instance_path = '../../mt_vrp_o/data/06_09_2026/mt_vrp_o_instances_fast_targets_nonlinear//targ18_win50_random_seed0_occprob0.2_vmaxa4.0_3agents_vmint4.0_vmaxt6.0/'
occupancy = np.load(instance_path + '/occupancy.npy')

map_save_path = repo_path + '/data/tmp/'

home = os.path.expanduser("~")
polyanya_path = home + '/polyanya/anyangle/polyanya/'

polyanya_wrapper = PolyanyaWrapper(occupancy.astype(bool).transpose(), \
                                   polyanya_path, \
                                   map_save_path)

# The case that fails if I have EPSILON in poly_contains_point and get_point_location
start = np.array([6.04640611378642401, 25.00000000271804623])
goal = np.array([15.00142199751917538, 0.03432802248710658])

plan_path = True

if plan_path:
  print('Planning path')
  path = polyanya_wrapper.shortest_path(start, goal)
  print(path)

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.patches as patches
matplotlib.rcParams.update({'font.size': 45})
prop_cycle = plt.rcParams['axes.prop_cycle']
prop_cycle_colors = prop_cycle.by_key()['color']

plt.figure(figsize=(15,15))
ax = plt.gca()

from png_to_boxes import im_to_rects, rects_to_polytopes, get_shared_rect_boundaries, get_connected_rect_component, merge_rects_into_polygons
obstacle_rects = im_to_rects(np.copy(occupancy.reshape(*occupancy.shape, 1)))
obstacle_color = 'chocolate'
for rect_idx in range(len(obstacle_rects)):
  rect = obstacle_rects[rect_idx]
  xy = (rect[0][0], rect[0][1])
  width = rect[1][0] - xy[0]
  height = rect[1][1] - xy[1]
  ax.add_patch(patches.Rectangle(xy, width, height, facecolor=obstacle_color))

ax.scatter([start[0]], [start[1]], color='red', label='start position')
ax.scatter([goal[0]], [goal[1]], color='aqua', label='goal position')
if plan_path:
  ax.plot(path[:, 0], path[:, 1], color='blue', linewidth=4, solid_capstyle='butt')
# ax.plot([7, 11], [25, 25], color='blue', linewidth=4, solid_capstyle='butt')

with open('polygons.txt', 'r') as f:
  lines = f.readlines()
polygon_idx = -1
polygons = []
for line in lines:
  if 'polygon ' + str(polygon_idx + 1) in line:
    polygon_idx += 1
    polygons.append([])
    continue

  point = [int(i) for i in line[1:-2].split(',')]
  polygons[-1].append(point)

for polygon_idx in range(len(polygons)):
  polygons[polygon_idx].append(polygons[polygon_idx][-1])
  polygons[polygon_idx] = np.array(polygons[polygon_idx])
  ax.plot(polygons[polygon_idx][:, 0], polygons[polygon_idx][:, 1], color='fuchsia')

plt.xlim(0, occupancy.shape[0])
plt.ylim(0, occupancy.shape[1])
plt.legend(fontsize=15)
ax.set_aspect('equal')
plt.show()
