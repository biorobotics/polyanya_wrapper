import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull
import shapely
from geometry_utils import *

# Removes holes from polygons
def merge_rects_into_polygons(rects, allow_squeeze=False, use_adj_list=False, verbose=False):
  num_rects = len(rects)
  if use_adj_list:
    adj_list = [[] for rect in rects]
  else:
    adj_mat = np.zeros((num_rects, num_rects))

  use_shapely = True
  for rect_idx1, rect1 in enumerate(rects):
    for rect_idx2_local, rect2 in enumerate(rects[rect_idx1 + 1:]):
      rect_idx2 = rect_idx1 + 1 + rect_idx2_local

      if verbose:
        print('Getting rectangle adjacency graph: rect_idx1 = %d of %d, rect_idx2 = %d of %d' %(rect_idx1, len(rects), rect_idx2, len(rects)))

      if use_shapely:
        if shapely.intersects(shapely.box(*rect1[0], *rect1[1]), shapely.box(*rect2[0], *rect2[1])):
          if use_adj_list:
            adj_list[rect_idx1].append(rect_idx2)
            adj_list[rect_idx2].append(rect_idx1)
          else:
            adj_mat[rect_idx1, rect_idx2] = 1
            adj_mat[rect_idx2, rect_idx1] = 1
      else:
        upper_left1 = rect1[0]
        lower_right1 = rect1[1]
        xstart1 = upper_left1[0]
        xend1 = lower_right1[0]
        ystart1 = upper_left1[1]
        yend1 = lower_right1[1]

        upper_left2 = rect2[0]
        lower_right2 = rect2[1]
        xstart2 = upper_left2[0]
        xend2 = lower_right2[0]
        ystart2 = upper_left2[1]
        yend2 = lower_right2[1]

        if xstart1 == xend2 or xstart2 == xend1:
          if xstart1 == xend2:
            # -x side of rect1 meets +x side of rect2
            xstart = xstart1
            xend = xstart
          else:
            # -x side of rect2 meets +x side of rect1
            xstart = xstart2
            xend = xstart

          if ystart1 < ystart2:
            ystart = ystart2
          else:
            ystart = ystart1

          if yend1 < yend2:
            yend = yend1
          else:
            yend = yend2

          if yend < ystart:
            continue
        elif ystart1 == yend2 or ystart2 == yend1:
          if ystart1 == yend2:
            # -y side of rect1 meets +y side of rect2
            ystart = ystart1
            yend = ystart
          else:
            # -y side of rect2 meets +y side of rect1
            ystart = ystart2
            yend = ystart

          if xstart1 < xstart2:
            xstart = xstart2
          else:
            xstart = xstart1

          if xend1 < xend2:
            xend = xend1
          else:
            xend = xend2

          if xend < xstart:
            continue
        else:
          continue

        if not allow_squeeze and xstart == xend and ystart == yend:
          continue

        if use_adj_list:
          adj_list[rect_idx1].append(rect_idx2)
          adj_list[rect_idx2].append(rect_idx1)
        else:
          adj_mat[rect_idx1, rect_idx2] = 1
          adj_mat[rect_idx2, rect_idx1] = 1

  stack = []
  rect_group_idx = [-1 for rect_idx in range(num_rects)]
  rect_groups = []
  group_idx = 0
  for rect_idx1 in range(num_rects):
    if rect_group_idx[rect_idx1] != -1:
      continue

    rect_groups.append([])
    stack.append(rect_idx1)
    while len(stack) != 0:
      rect_idx2 = stack.pop()
      rect_group_idx[rect_idx2] = group_idx
      rect_groups[-1].append(rects[rect_idx2])
      if use_adj_list:
        stack.extend([rect_idx3 for rect_idx3 in adj_list[rect_idx2] if rect_group_idx[rect_idx3] == -1])
      else:
        stack.extend([rect_idx3 for rect_idx3, adj in enumerate(adj_mat[rect_idx2]) if adj and rect_group_idx[rect_idx3] == -1])

    group_idx += 1

  # For each group, merge the rectangles
  polys = []
  for group in rect_groups:
    rect1 = group[0]
    poly1 = shapely.normalize(shapely.box(*rect1[0], *rect1[1]))
    for rect2 in group[1:]:
      poly2 = shapely.box(*rect2[0], *rect2[1])
      poly1 = shapely.normalize(shapely.union(poly1, poly2))
    coords = np.array([coord for coord in poly1.exterior.coords])
    poly = shapely.Polygon(coords)
    polys.append(poly)

  return polys

def get_connected_rect_component(rects, root_rect_idx, allow_squeeze=False):
  num_rects = len(rects)
  adj_mat = np.zeros((num_rects, num_rects))

  for rect_idx1, rect1 in enumerate(rects):
    for rect_idx2_local, rect2 in enumerate(rects[rect_idx1 + 1:]):
      rect_idx2 = rect_idx1 + 1 + rect_idx2_local

      upper_left1 = rect1[0]
      lower_right1 = rect1[1]
      xstart1 = upper_left1[0]
      xend1 = lower_right1[0]
      ystart1 = upper_left1[1]
      yend1 = lower_right1[1]

      upper_left2 = rect2[0]
      lower_right2 = rect2[1]
      xstart2 = upper_left2[0]
      xend2 = lower_right2[0]
      ystart2 = upper_left2[1]
      yend2 = lower_right2[1]

      if xstart1 == xend2 or xstart2 == xend1:
        if xstart1 == xend2:
          # -x side of rect1 meets +x side of rect2
          xstart = xstart1
          xend = xstart
        else:
          # -x side of rect2 meets +x side of rect1
          xstart = xstart2
          xend = xstart

        if ystart1 < ystart2:
          ystart = ystart2
        else:
          ystart = ystart1

        if yend1 < yend2:
          yend = yend1
        else:
          yend = yend2

        if yend < ystart:
          continue
      elif ystart1 == yend2 or ystart2 == yend1:
        if ystart1 == yend2:
          # -y side of rect1 meets +y side of rect2
          ystart = ystart1
          yend = ystart
        else:
          # -y side of rect2 meets +y side of rect1
          ystart = ystart2
          yend = ystart

        if xstart1 < xstart2:
          xstart = xstart2
        else:
          xstart = xstart1

        if xend1 < xend2:
          xend = xend1
        else:
          xend = xend2

        if xend < xstart:
          continue
      else:
        continue

      if not allow_squeeze and xstart == xend and ystart == yend:
        continue

      adj_mat[rect_idx1, rect_idx2] = 1
      adj_mat[rect_idx2, rect_idx1] = 1

  closed_list = set()
  stack = [root_rect_idx]
  while len(stack) != 0:
    rect_idx = stack.pop()
    closed_list.add(rect_idx)
    stack.extend([rect_idx2 for rect_idx2, adj in enumerate(adj_mat[rect_idx]) if adj and rect_idx2 not in closed_list])

  return closed_list

def convex_hulls_of_merged_rects(rects):
  num_rects = len(rects)
  adj_mat = np.zeros((num_rects, num_rects))

  for rect_idx1, rect1 in enumerate(rects):
    for rect_idx2_local, rect2 in enumerate(rects[rect_idx1 + 1:]):
      rect_idx2 = rect_idx1 + 1 + rect_idx2_local

      upper_left1 = rect1[0]
      lower_right1 = rect1[1]
      xstart1 = upper_left1[0]
      xend1 = lower_right1[0]
      ystart1 = upper_left1[1]
      yend1 = lower_right1[1]

      upper_left2 = rect2[0]
      lower_right2 = rect2[1]
      xstart2 = upper_left2[0]
      xend2 = lower_right2[0]
      ystart2 = upper_left2[1]
      yend2 = lower_right2[1]

      if xstart1 == xend2 or xstart2 == xend1:
        if xstart1 == xend2:
          # -x side of rect1 meets +x side of rect2
          xstart = xstart1
          xend = xstart
        else:
          # -x side of rect2 meets +x side of rect1
          xstart = xstart2
          xend = xstart

        if ystart1 < ystart2:
          ystart = ystart2
        else:
          ystart = ystart1

        if yend1 < yend2:
          yend = yend1
        else:
          yend = yend2

        if yend < ystart:
          continue
      elif ystart1 == yend2 or ystart2 == yend1:
        if ystart1 == yend2:
          # -y side of rect1 meets +y side of rect2
          ystart = ystart1
          yend = ystart
        else:
          # -y side of rect2 meets +y side of rect1
          ystart = ystart2
          yend = ystart

        if xstart1 < xstart2:
          xstart = xstart2
        else:
          xstart = xstart1

        if xend1 < xend2:
          xend = xend1
        else:
          xend = xend2

        if xend < xstart:
          continue
      else:
        continue

      adj_mat[rect_idx1, rect_idx2] = 1
      adj_mat[rect_idx2, rect_idx1] = 1

  stack = []
  rect_group_idx = [-1 for rect_idx in range(num_rects)]
  rect_groups = []
  group_idx = 0
  for rect_idx1 in range(num_rects):
    if rect_group_idx[rect_idx1] != -1:
      continue

    rect_groups.append([])
    stack.append(rect_idx1)
    while len(stack) != 0:
      rect_idx2 = stack.pop()
      rect_group_idx[rect_idx2] = group_idx
      rect_groups[-1].append(rects[rect_idx2])
      stack.extend([rect_idx3 for rect_idx3, adj in enumerate(adj_mat[rect_idx2]) if adj and rect_group_idx[rect_idx3] == -1])

    group_idx += 1

  chulls = []
  for group in rect_groups:
    points = []
    for rect in group:
      upper_left = rect[0]
      lower_right = rect[1]

      xstart = upper_left[0]
      xend = lower_right[0]
      ystart = upper_left[1]
      yend = lower_right[1]

      points.extend([(xstart, ystart), (xend, ystart), (xend, yend), (xstart, yend)])

    points = np.array(points)
    chull = ConvexHull(points)
    chulls.append(points[chull.vertices])

  return chulls

def get_shared_rect_boundaries(rects, allow_squeeze=False):
  shared_boundaries = []
  shared_boundaries_ptr = [[] for rect in rects]
  shared_boundary_to_rects_ptr = []
  for rect_idx1, rect1 in enumerate(rects):
    for rect_idx2_local, rect2 in enumerate(rects[rect_idx1 + 1:]):
      rect_idx2 = rect_idx1 + 1 + rect_idx2_local

      upper_left1 = rect1[0]
      lower_right1 = rect1[1]
      xstart1 = upper_left1[0]
      xend1 = lower_right1[0]
      ystart1 = upper_left1[1]
      yend1 = lower_right1[1]

      upper_left2 = rect2[0]
      lower_right2 = rect2[1]
      xstart2 = upper_left2[0]
      xend2 = lower_right2[0]
      ystart2 = upper_left2[1]
      yend2 = lower_right2[1]

      if xstart1 == xend2 or xstart2 == xend1:
        if xstart1 == xend2:
          # -x side of rect1 meets +x side of rect2
          xstart = xstart1
          xend = xstart
        else:
          # -x side of rect2 meets +x side of rect1
          xstart = xstart2
          xend = xstart

        if ystart1 < ystart2:
          ystart = ystart2
        else:
          ystart = ystart1

        if yend1 < yend2:
          yend = yend1
        else:
          yend = yend2

        if yend < ystart:
          continue
      elif ystart1 == yend2 or ystart2 == yend1:
        if ystart1 == yend2:
          # -y side of rect1 meets +y side of rect2
          ystart = ystart1
          yend = ystart
        else:
          # -y side of rect2 meets +y side of rect1
          ystart = ystart2
          yend = ystart

        if xstart1 < xstart2:
          xstart = xstart2
        else:
          xstart = xstart1

        if xend1 < xend2:
          xend = xend1
        else:
          xend = xend2

        if xend < xstart:
          continue
      else:
        continue

      if not allow_squeeze and xstart == xend and ystart == yend:
        continue

      shared_boundaries_ptr[rect_idx1].append(len(shared_boundaries))
      shared_boundaries_ptr[rect_idx2].append(len(shared_boundaries))
      shared_boundaries.append(np.array([xstart, xend, ystart, yend]))
      shared_boundary_to_rects_ptr.append((rect_idx1, rect_idx2))

  if len(shared_boundaries) == 0:
    return np.zeros((0, 4)), shared_boundaries_ptr, np.zeros((0, 2), dtype=int)
  return np.array(shared_boundaries), shared_boundaries_ptr, np.array(shared_boundary_to_rects_ptr)

def get_shared_rect_boundary_segments(rects, allow_squeeze=False, max_segment_length=np.nan):
  shared_boundaries = []
  shared_boundaries_ptr = [[] for rect in rects]
  shared_boundary_ids = [] # Boundaries share an id if they are broken-up pieces of a larger boundary
  next_group_id = 0
  for rect_idx1, rect1 in enumerate(rects):
    for rect_idx2_local, rect2 in enumerate(rects[rect_idx1 + 1:]):
      rect_idx2 = rect_idx1 + 1 + rect_idx2_local

      upper_left1 = rect1[0]
      lower_right1 = rect1[1]
      xstart1 = upper_left1[0]
      xend1 = lower_right1[0]
      ystart1 = upper_left1[1]
      yend1 = lower_right1[1]

      upper_left2 = rect2[0]
      lower_right2 = rect2[1]
      xstart2 = upper_left2[0]
      xend2 = lower_right2[0]
      ystart2 = upper_left2[1]
      yend2 = lower_right2[1]

      if xstart1 == xend2 or xstart2 == xend1:
        if xstart1 == xend2:
          # -x side of rect1 meets +x side of rect2
          xstart = xstart1
          xend = xstart
        else:
          # -x side of rect2 meets +x side of rect1
          xstart = xstart2
          xend = xstart

        if ystart1 < ystart2:
          ystart = ystart2
        else:
          ystart = ystart1

        if yend1 < yend2:
          yend = yend1
        else:
          yend = yend2

        if yend < ystart:
          continue
      elif ystart1 == yend2 or ystart2 == yend1:
        if ystart1 == yend2:
          # -y side of rect1 meets +y side of rect2
          ystart = ystart1
          yend = ystart
        else:
          # -y side of rect2 meets +y side of rect1
          ystart = ystart2
          yend = ystart

        if xstart1 < xstart2:
          xstart = xstart2
        else:
          xstart = xstart1

        if xend1 < xend2:
          xend = xend1
        else:
          xend = xend2

        if xend < xstart:
          continue
      else:
        continue

      if not allow_squeeze and xstart == xend and ystart == yend:
        continue

      if np.isnan(max_segment_length):
        shared_boundaries_ptr[rect_idx1].append(len(shared_boundaries))
        shared_boundaries_ptr[rect_idx2].append(len(shared_boundaries))
        shared_boundaries.append(np.array([xstart, xend, ystart, yend]))
        shared_boundary_ids.append(next_group_id)
      else:
        delta_x = xend - xstart
        delta_y = yend - ystart
        xend - xstart
        length = max(delta_x, delta_y)
        num_segments = int(np.ceil(length/max_segment_length))
        x_segment_length = min(delta_x, max_segment_length)
        y_segment_length = min(delta_y, max_segment_length)
        for segment_idx in range(num_segments):
          shared_boundaries_ptr[rect_idx1].append(len(shared_boundaries))
          shared_boundaries_ptr[rect_idx2].append(len(shared_boundaries))
          shared_boundaries.append(np.array([xstart + segment_idx*x_segment_length, \
                                             min(xstart + (segment_idx + 1)*x_segment_length, xend), \
                                             ystart + segment_idx*y_segment_length, \
                                             min(yend + (segment_idx + 1)*y_segment_length, yend)]))
          shared_boundary_ids.append(next_group_id)
      next_group_id += 1

  if len(shared_boundaries) == 0:
    return np.zeros((0, 4)), shared_boundaries_ptr, shared_boundary_ids
  return np.array(shared_boundaries), shared_boundaries_ptr, np.array(shared_boundary_ids)

def get_rect_adj_mat(rects, allow_squeeze=False):
  adj_mat = np.zeros((len(rects), len(rects)), dtype=bool)
  for rect_idx1, rect1 in enumerate(rects):
    for rect_idx2_local, rect2 in enumerate(rects[rect_idx1 + 1:]):
      rect_idx2 = rect_idx1 + 1 + rect_idx2_local

      upper_left1 = rect1[0]
      lower_right1 = rect1[1]
      xstart1 = upper_left1[0]
      xend1 = lower_right1[0]
      ystart1 = upper_left1[1]
      yend1 = lower_right1[1]

      upper_left2 = rect2[0]
      lower_right2 = rect2[1]
      xstart2 = upper_left2[0]
      xend2 = lower_right2[0]
      ystart2 = upper_left2[1]
      yend2 = lower_right2[1]

      if xstart1 == xend2 or xstart2 == xend1:
        if xstart1 == xend2:
          # -x side of rect1 meets +x side of rect2
          xstart = xstart1
          xend = xstart
        else:
          # -x side of rect2 meets +x side of rect1
          xstart = xstart2
          xend = xstart

        if ystart1 < ystart2:
          ystart = ystart2
        else:
          ystart = ystart1

        if yend1 < yend2:
          yend = yend1
        else:
          yend = yend2

        if yend < ystart:
          continue
      elif ystart1 == yend2 or ystart2 == yend1:
        if ystart1 == yend2:
          # -y side of rect1 meets +y side of rect2
          ystart = ystart1
          yend = ystart
        else:
          # -y side of rect2 meets +y side of rect1
          ystart = ystart2
          yend = ystart

        if xstart1 < xstart2:
          xstart = xstart2
        else:
          xstart = xstart1

        if xend1 < xend2:
          xend = xend1
        else:
          xend = xend2

        if xend < xstart:
          continue
      else:
        continue

      if not allow_squeeze and xstart == xend and ystart == yend:
        continue

      adj_mat[rect_idx1, rect_idx2] = True
      adj_mat[rect_idx2, rect_idx1] = True

  return adj_mat

def rects_to_polytopes(rects):
  polytopes = []
  A = np.zeros((4, 2))
  A[0, 0] = 1
  A[1, 1] = 1
  A[2, 0] = -1
  A[3, 1] = -1
  for rect in rects:
    b = np.zeros(4)

    upper_left, lower_right = rect

    b[0] = -lower_right[0]
    b[1] = -lower_right[1]
    b[2] = upper_left[0]
    b[3] = upper_left[1]

    polytopes.append((np.copy(A), b))

  return polytopes

# Converts an image to a bunch of 2D rectangles in pixel coordinates
def im_to_rects(im):
  rects = [] # Stores upper-left (inclusive) and lower-right (exclusive) corners of rectangle (where upper corresponds to lower number rows)
  for row in range(im.shape[0]):
    for col in range(im.shape[1]):
      # If this pixel doesn't already belong to a rectangle, determine the maximal containing rectangle
      if im[row, col, 0] == 255:
        upper_left = (row, col)

        # Find column of lower-right corner (exclusive)
        lower_right_col = col + 1
        for pcol in range(col + 1, im.shape[1]):
          lower_right_col = pcol
          if im[row, pcol, 0] != 255:
            break
          elif pcol == im.shape[1] - 1:
            lower_right_col = im.shape[1]

        # Find row of lower-right corner (exclusive)
        lower_right_row = row + 1
        for prow in range(row + 1, im.shape[0]):
          lower_right_row = prow
          found = False
          for pcol in range(col, lower_right_col):
            if im[prow, pcol, 0] != 255:
              found = True
              break

          if found:
            break
          elif prow == im.shape[0] - 1:
            lower_right_row = im.shape[0]

        lower_right = (lower_right_row, lower_right_col)

        rects.append((upper_left, lower_right))

        # Fill the rectangle
        for prow in range(upper_left[0], lower_right[0]):
          for pcol in range(upper_left[1], lower_right[1]):
            im[prow, pcol, :] = 0

  return rects

def png_to_rects(image_path):
  return im_to_rects(cv2.imread(image_path))

def png_to_boxes(image_path, cell_size, floor_z, ceiling_z, map_lb, map_ub):
  rects = png_to_rects(image_path)
  boxes = []
  im = cv2.imread(image_path)
  for rect in rects:
    cx = cell_size[0]*(rect[0][1] + rect[1][1])/2 + map_lb[0]
    cy = cell_size[1]*(rect[0][0] + rect[1][0])/2 + map_lb[1]
    cz = (floor_z + ceiling_z)/2
    sx = cell_size[0]*(rect[1][1] - rect[0][1])
    sy = cell_size[1]*(rect[1][0] - rect[0][0])
    sz = ceiling_z - floor_z
    boxes.append(np.array([cx, cy, cz, sx, sy, sz]))

  # Floor
  cx = cell_size[0]*im.shape[1]/2 + map_lb[0]
  cy = cell_size[1]*im.shape[0]/2 + map_lb[1]
  cz = (map_lb[2] + floor_z)/2
  sx = cell_size[0]*im.shape[1]
  sy = cell_size[1]*im.shape[0]
  sz = floor_z - map_lb[2]
  boxes.append(np.array([cx, cy, cz, sx, sy, sz]))

  # Ceiling
  cz = (map_ub[2] + ceiling_z)/2
  sz = map_ub[2] - ceiling_z
  boxes.append(np.array([cx, cy, cz, sx, sy, sz]))

  return boxes
