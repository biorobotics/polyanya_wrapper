import numpy as np
import time

from png_to_boxes import im_to_rects, rects_to_polytopes, get_shared_rect_boundaries, get_connected_rect_component, merge_rects_into_polygons

from geometry_utils import *

import sys

# For visibility 
sys.path.append('../build/')

import pickle

from multiprocessing import Process, Queue, Pool

from visibility_bindings import VisibilityWrapper
import shapely

import networkx as nx
from scipy.sparse import coo_matrix

np.set_printoptions(linewidth=np.inf)

repo_path = '../'

use_multiprocessing = True

def vgraph_proc_fn(arg): 
  row_poly_shapely, row_poly, row, vertex_locs = arg
  vgraph_rows = []
  vgraph_cols = []
  vgraph_data = [] # Edge costs
  coords = np.concatenate((row_poly[0:1, :2], row_poly[:, 2:]), axis=0)
  for col in range(row + 1, len(vertex_locs)):
    # Prevent squeezing
    if np.all(vertex_locs[row] == vertex_locs[col]):
      continue

    if not np.any(np.logical_and(coords[:, 0] == vertex_locs[col, 0], coords[:, 1] == vertex_locs[col, 1])):
      continue

    vgraph_rows.append(row)
    vgraph_cols.append(col)
    vgraph_data.append(np.linalg.norm(vertex_locs[row] - vertex_locs[col]))

    vgraph_rows.append(col)
    vgraph_cols.append(row)
    vgraph_data.append(vgraph_data[-1])

  return vgraph_rows, vgraph_cols, vgraph_data

def construct_vg(obstacles_info, time_limit):
  overall_timer_start = time.perf_counter()

  occupancy, map_lb, map_ub = obstacles_info

  # Compute visibility polygons
  bt = time.perf_counter()
  num_cells_x = occupancy.shape[0]
  num_cells_y = occupancy.shape[1]
  cell_size_x = (map_ub[0] - map_lb[0])/num_cells_x
  cell_size_y = (map_ub[1] - map_lb[1])/num_cells_y
  obstacle_rects_int = im_to_rects(np.copy(occupancy.reshape(num_cells_x, num_cells_y, 1)))
  obstacle_rects = [((rect[0][0]*cell_size_x + map_lb[0], rect[0][1]*cell_size_y + map_lb[1]), (rect[1][0]*cell_size_x + map_lb[0], rect[1][1]*cell_size_y + map_lb[1])) for rect in obstacle_rects_int] # Account for spatial extent of cells
  # Inflate rects to avoid squeezing
  eps = 1e-7
  obstacle_rects = [((rect[0][0] - eps, rect[0][1] - eps), (rect[1][0] + eps, rect[1][1] + eps)) for rect in obstacle_rects]

  # Obstacles can't intersect boundary or CGAL throws an error
  boundary_vertices = np.array([[map_lb[0] - cell_size_x, map_lb[1] - cell_size_y], \
                                [map_ub[0] + cell_size_x, map_lb[1] - cell_size_y], \
                                [map_ub[0] + cell_size_x, map_ub[1] + cell_size_y], \
                                [map_lb[0] - cell_size_x, map_ub[1] + cell_size_y], \
                                [map_lb[0] - cell_size_x, map_lb[1] - cell_size_y]]).astype(np.float64)
  obs_polys = merge_rects_into_polygons(obstacle_rects)
  obstacle_vertices = []
  obstacle_start_indices = []
  is_concave = []
  start_idx = 0
  for poly in obs_polys:
    obstacle_start_indices.append(start_idx)
    coords = np.array([coord for coord in poly.exterior.coords])
    obstacle_vertices.append(coords)
    start_idx += coords.shape[0]
    coord1 = coords[-2]
    coord2 = coords[-1]
    coord3 = coords[1]
    start_coord_is_concave = np.cross(coord2 - coord1, coord3 - coord2) > 0
    is_concave.append(start_coord_is_concave)
    for coord1, coord2, coord3 in zip(coords[:-2], coords[1:-1], coords[2:]):
      is_concave.append(np.cross(coord2 - coord1, coord3 - coord2) > 0)
    is_concave.append(start_coord_is_concave)
      
  obstacle_start_indices = np.array(obstacle_start_indices, dtype=int)
  if len(obstacle_vertices):
    obstacle_vertices = np.concatenate(obstacle_vertices, axis=0)
  else:
    obstacle_vertices = np.zeros((0, 2))

  vertex_to_obstacle_ptr = []
  for i in range(len(obstacle_start_indices)):
    start_idx = obstacle_start_indices[i]
    if i == len(obstacle_start_indices) - 1:
      end_idx = len(obstacle_vertices)
    else:
      end_idx = obstacle_start_indices[i + 1]

    for j in range(start_idx, end_idx - 1):
      vertex_to_obstacle_ptr.append(i)
    vertex_to_obstacle_ptr.append(i)

  visibility_wrapper = VisibilityWrapper(boundary_vertices, obstacle_vertices, obstacle_start_indices)
  vertex_locs = []
  vpolys = []
  vpolys_shapely = []
  for obstacle_vertex_idx in range(len(obstacle_vertices)):
    # Compute a single visibility polygon
    if obstacle_vertex_idx in obstacle_start_indices:
      continue
    if obstacle_vertices[obstacle_vertex_idx][0] <= map_lb[0] or obstacle_vertices[obstacle_vertex_idx][0] >= map_ub[0] or obstacle_vertices[obstacle_vertex_idx][1] <= map_lb[1] or obstacle_vertices[obstacle_vertex_idx][1] >= map_ub[1]:
      continue
    if is_concave[obstacle_vertex_idx]:
      continue
    vpoly = visibility_wrapper.visibility_polygon(obstacle_vertex_idx)
    vertex_locs.append(obstacle_vertices[obstacle_vertex_idx])
    vpolys.append(vpoly)
    coords = np.concatenate((vpoly[0:1, :2], vpoly[:, 2:]), axis=0)
    vpolys_shapely.append(shapely.geometry.Polygon(coords))
  at = time.perf_counter()
  print('Time to compute all vpolys', at - bt)
  vpoly_time = at - bt

  vertex_locs = np.array(vertex_locs)

  # Build visibility graph. Nodes are obstacle vertices
  bt = time.perf_counter()
  vgraph_rows = []
  vgraph_cols = []
  vgraph_data = [] # Edge costs
  num_vgraph_nodes = len(vertex_locs)
  if use_multiprocessing:
    num_proc = 8
    with Pool(processes=num_proc) as pool:
      vgraphs_per_proc = pool.map(vgraph_proc_fn, [(vpolys_shapely[row], vpolys[row], row, vertex_locs) for row in range(len(vertex_locs))])

    vgraph_rows = np.concatenate([vgraph[0] for vgraph in vgraphs_per_proc if len(vgraph[0]) != 0])
    vgraph_cols = np.concatenate([vgraph[1] for vgraph in vgraphs_per_proc if len(vgraph[0]) != 0])
    vgraph_data = np.concatenate([vgraph[2] for vgraph in vgraphs_per_proc if len(vgraph[0]) != 0])
  else:
    print('Non multi-processed version of vgraph construction not implemented')
    quit()

  vgraph_scipy = coo_matrix((vgraph_data, (vgraph_rows, vgraph_cols)), (num_vgraph_nodes, num_vgraph_nodes)).tocsr()
  vgraph_indptr = [i for i in vgraph_scipy.indptr]
  vgraph_indices = [i for i in vgraph_scipy.indices]
  vgraph_data = [d for d in vgraph_scipy.data]
  vgraph = nx.from_scipy_sparse_array(vgraph_scipy, create_using=nx.DiGraph)

  at = time.perf_counter()
  vgraph_time = at - bt
  print('Time for vgraph: %f' %(vgraph_time))

  return vgraph_indptr, vgraph_indices, vgraph_data, vertex_locs, visibility_wrapper
