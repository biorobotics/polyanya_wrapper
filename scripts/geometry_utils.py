import numpy as np

# https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
def dist_point_to_line_segment(x1, y1, x2, y2, x3, y3): # x3,y3 is the point
  px = x2-x1
  py = y2-y1

  norm = px*px + py*py

  if norm == 0:
    return np.sqrt((x3 - x2)**2 + (y3 - y2)**2)

  u =  ((x3 - x1) * px + (y3 - y1) * py) / float(norm)

  if u > 1:
      u = 1
  elif u < 0:
      u = 0

  x = x1 + u * px
  y = y1 + u * py

  dx = x - x3
  dy = y - y3

  # Note: If the actual distance does not matter,
  # if you only want to compare what this function
  # returns to other results of this function, you
  # can just return the squared distance instead
  # (i.e. remove the sqrt) to gain a little performance

  dist = (dx*dx + dy*dy)**.5

  return dist

# https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
def get_closest_point_on_line_segment(x1, y1, x2, y2, x3, y3): # x3,y3 is the point
  px = x2-x1
  py = y2-y1

  norm = px*px + py*py

  if norm == 0:
    return x2, y2, np.sqrt((x3 - x2)**2 + (y3 - y2)**2)

  u =  ((x3 - x1) * px + (y3 - y1) * py) / float(norm)

  if u > 1:
      u = 1
  elif u < 0:
      u = 0

  x = x1 + u * px
  y = y1 + u * py

  dx = x - x3
  dy = y - y3

  # Note: If the actual distance does not matter,
  # if you only want to compare what this function
  # returns to other results of this function, you
  # can just return the squared distance instead
  # (i.e. remove the sqrt) to gain a little performance

  dist = (dx*dx + dy*dy)**.5

  return x, y, dist

# https://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
def get_closest_point_on_line_segment_and_percent(x1, y1, x2, y2, x3, y3): # x3,y3 is the point
  px = x2-x1
  py = y2-y1

  norm = px*px + py*py

  if norm == 0:
    return x2, y2, np.sqrt((x3 - x2)**2 + (y3 - y2)**2)

  u =  ((x3 - x1) * px + (y3 - y1) * py) / float(norm)

  if u > 1:
      u = 1
  elif u < 0:
      u = 0

  x = x1 + u * px
  y = y1 + u * py

  dx = x - x3
  dy = y - y3

  # Note: If the actual distance does not matter,
  # if you only want to compare what this function
  # returns to other results of this function, you
  # can just return the squared distance instead
  # (i.e. remove the sqrt) to gain a little performance

  dist = (dx*dx + dy*dy)**.5

  return x, y, dist, u

def dist_point_to_line_segment_3d(p0, p1, pquery, return_percent=False): # p0 and p1 are endpoints of the segment, pquery is the query point
  unnorm_direction = p1 - p0
  norm = np.linalg.norm(unnorm_direction)
  if norm == 0:
    return np.linalg.norm(pquery - p0)
  direction = unnorm_direction/norm

  prel = pquery - p0
  dot = direction@prel
  if dot > norm:
    if return_percent:
      return np.linalg.norm(pquery - p1), 1.
    return np.linalg.norm(pquery - p1)
  elif dot < 0:
    if return_percent:
      return np.linalg.norm(pquery - p0), 0.
    return np.linalg.norm(pquery - p0)

  if return_percent:
    return np.linalg.norm(prel - dot*direction), dot/norm

  return np.linalg.norm(prel - dot*direction)

'''
# Given three collinear points p, q, r, the function checks if
# point q lies on line segment 'pr'
def onSegment(p, q, r): 
  if (q[0] <= max(p[0], r[0])) and (q[0] >= min(p[0], r[0])) and \
     (q[1] <= max(p[1], r[1])) and (q[1] >= min(p[1], r[1])):
    return True
  return False
'''
# Points don't have to be collinear
def onSegment(p, q, r): 
  return dist_point_to_line_segment(*p, *r, *q) == 0.

def lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                          x2i, y2i, x2f, y2f):
  a = x1f - x1i
  b = x2i - x2f
  c = y1f - y1i
  d = y2i - y2f

  det = a*d - b*c

  if det == 0:
    # Segments are parallel
    return onSegment((x1i, y1i), (x2i, y2i), (x1f, y1f)) or \
           onSegment((x1i, y1i), (x2f, y2f), (x1f, y1f)) or \
           onSegment((x2i, y2i), (x1i, y1i), (x2f, y2f)) or \
           onSegment((x2i, y2i), (x1f, y1f), (x2f, y2f))

  lx = x2i - x1i
  ly = y2i - y1i

  t1 = (d*lx - b*ly)/det
  t2 = (-c*lx + a*ly)/det
  return 0 <= t1 and t1 <= 1 and 0 <= t2 and t2 <= 1

def lineSegmentsIntersect_round(x1i, y1i, x1f, y1f, \
                                x2i, y2i, x2f, y2f, ndigits=2):
  a = x1f - x1i
  b = x2i - x2f
  c = y1f - y1i
  d = y2i - y2f

  det = a*d - b*c

  if det == 0:
    # Segments are parallel
    return onSegment((x1i, y1i), (x2i, y2i), (x1f, y1f)) or \
           onSegment((x1i, y1i), (x2f, y2f), (x1f, y1f)) or \
           onSegment((x2i, y2i), (x1i, y1i), (x2f, y2f)) or \
           onSegment((x2i, y2i), (x1f, y1f), (x2f, y2f))

  lx = x2i - x1i
  ly = y2i - y1i

  t1 = round((d*lx - b*ly)/det, ndigits)
  t2 = round((-c*lx + a*ly)/det, ndigits)
  return 0 <= t1 and t1 <= 1 and 0 <= t2 and t2 <= 1

def lineSegmentsIntersectPercent(x1i, y1i, x1f, y1f, \
                                 x2i, y2i, x2f, y2f, first):
  a = x1f - x1i
  b = x2i - x2f
  c = y1f - y1i
  d = y2i - y2f

  det = a*d - b*c

  if det == 0:
    # Segments are parallel
    if first:
      if onSegment((x1i, y1i), (x2i, y2i), (x1f, y1f)):
        return True, np.linalg.norm([x2i - x1i, y2i - y1i])/np.linalg.norm([x1f - x1i, y1f - y1i])
      elif onSegment((x1i, y1i), (x2f, y2f), (x1f, y1f)):
        return True, np.linalg.norm([x2f - x1i, y2f - y1i])/np.linalg.norm([x1f - x1i, y1f - y1i])
      elif onSegment((x2i, y2i), (x1i, y1i), (x2f, y2f)):
        return True, 0.
      elif onSegment((x2i, y2i), (x1f, y1f), (x2f, y2f)):
        return True, 1.
    else:
      if onSegment((x1i, y1i), (x2i, y2i), (x1f, y1f)):
        return True, 0.
      elif onSegment((x1i, y1i), (x2f, y2f), (x1f, y1f)):
        return True, 1.
      elif onSegment((x2i, y2i), (x1i, y1i), (x2f, y2f)):
        return True, np.linalg.norm([x1i - x2i, y1i - y2i])/np.linalg.norm([x2f - x2i, y2f - y2i])
      elif onSegment((x2i, y2i), (x1f, y1f), (x2f, y2f)):
        return True, np.linalg.norm([x1f - x2i, y1f - y2i])/np.linalg.norm([x2f - x2i, y2f - y2i])
    return False, 0.

  lx = x2i - x1i
  ly = y2i - y1i

  t1 = (d*lx - b*ly)/det
  t2 = (-c*lx + a*ly)/det
  return 0 <= t1 and t1 <= 1 and 0 <= t2 and t2 <= 1, t1 if first else t2

def lineSegmentsIntersectBothPercents(x1i, y1i, x1f, y1f, \
                                      x2i, y2i, x2f, y2f):
  a = x1f - x1i
  b = x2i - x2f
  c = y1f - y1i
  d = y2i - y2f

  det = a*d - b*c

  if det == 0:
    # Segments are parallel
    if onSegment((x1i, y1i), (x2i, y2i), (x1f, y1f)):
      return True, np.linalg.norm([x2i - x1i, y2i - y1i])/np.linalg.norm([x1f - x1i, y1f - y1i]), 0.
    elif onSegment((x1i, y1i), (x2f, y2f), (x1f, y1f)):
      return True, np.linalg.norm([x2f - x1i, y2f - y1i])/np.linalg.norm([x1f - x1i, y1f - y1i]), 1.
    elif onSegment((x2i, y2i), (x1i, y1i), (x2f, y2f)):
      return True, 0., np.linalg.norm([x1i - x2i, y1i - y2i])/np.linalg.norm([x2f - x2i, y2f - y2i])
    elif onSegment((x2i, y2i), (x1f, y1f), (x2f, y2f)):
      return True, 1., np.linalg.norm([x1f - x2i, y1f - y2i])/np.linalg.norm([x2f - x2i, y2f - y2i])
    return False, 0., 0.

  lx = x2i - x1i
  ly = y2i - y1i

  t1 = (d*lx - b*ly)/det
  t2 = (-c*lx + a*ly)/det
  return 0 <= t1 and t1 <= 1 and 0 <= t2 and t2 <= 1, t1, t2

def lineSegmentsIntersectPercent_round(x1i, y1i, x1f, y1f, \
                                       x2i, y2i, x2f, y2f, first, ndigits=2):
  a = x1f - x1i
  b = x2i - x2f
  c = y1f - y1i
  d = y2i - y2f

  det = a*d - b*c

  if det == 0:
    # Segments are parallel
    if first:
      if onSegment((x1i, y1i), (x2i, y2i), (x1f, y1f)):
        return True, np.linalg.norm([x2i - x1i, y2i - y1i])/np.linalg.norm([x1f - x1i, y1f - y1i])
      elif onSegment((x1i, y1i), (x2f, y2f), (x1f, y1f)):
        return True, np.linalg.norm([x2f - x1i, y2f - y1i])/np.linalg.norm([x1f - x1i, y1f - y1i])
      elif onSegment((x2i, y2i), (x1i, y1i), (x2f, y2f)):
        return True, 0.
      elif onSegment((x2i, y2i), (x1f, y1f), (x2f, y2f)):
        return True, 1.
    else:
      if onSegment((x1i, y1i), (x2i, y2i), (x1f, y1f)):
        return True, 0.
      elif onSegment((x1i, y1i), (x2f, y2f), (x1f, y1f)):
        return True, 1.
      elif onSegment((x2i, y2i), (x1i, y1i), (x2f, y2f)):
        return True, np.linalg.norm([x1i - x2i, y1i - y2i])/np.linalg.norm([x2f - x2i, y2f - y2i])
      elif onSegment((x2i, y2i), (x1f, y1f), (x2f, y2f)):
        return True, np.linalg.norm([x1f - x2i, y1f - y2i])/np.linalg.norm([x2f - x2i, y2f - y2i])
    return False, 0.

  lx = x2i - x1i
  ly = y2i - y1i

  t1 = round((d*lx - b*ly)/det, ndigits)
  t2 = round((-c*lx + a*ly)/det, ndigits)
  return 0 <= t1 and t1 <= 1 and 0 <= t2 and t2 <= 1, t1 if first else t2

def point_in_rect(x, y, \
                  xir, yir, xfr, yfr):
  return xir <= x <= xfr and \
         yir <= y <= yfr

def point_in_box(x, y, z, \
                 xir, yir, zir, xfr, yfr, zfr):
  return xir <= x <= xfr and \
         yir <= y <= yfr and \
         zir <= z <= zfr

def get_intersection_points_btw_line_segment_and_rect(xil, yil, xfl, yfl, \
                                                      xir, yir, xfr, yfr):
  rect_segs = [(xir, yir, xir, yfr), \
               (xir, yir, xfr, yir), \
               (xir, yfr, xfr, yfr), \
               (xfr, yir, xfr, yfr)]
  intersection_points = []
  for rect_seg in rect_segs:
    x1i, y1i, x1f, y1f = rect_seg
    x2i, y2i, x2f, y2f = xil, yil, xfl, yfl

    a = x1f - x1i
    b = x2i - x2f
    c = y1f - y1i
    d = y2i - y2f

    det = a*d - b*c

    if det == 0:
      # Segments are parallel. TODO: for the MT-TSP stuff, maybe I need to handle this differently
      assert(False)
      continue

    lx = x2i - x1i
    ly = y2i - y1i

    t1 = (d*lx - b*ly)/det
    t2 = (-c*lx + a*ly)/det
    if 0 <= t1 <= 1 and 0 <= t2 <= 1:
      intersection_points.append(((1 - t2)*np.array([xil, yil]) + t2*np.array([xfl, yfl]), t2))

  return intersection_points

# We assume the 1st 4 coordinates are for the line segment, 2nd 4 are for the ray
def rayIntersectsLineSegment(x1i, y1i, x1f, y1f, \
                             x2i, y2i, x2f, y2f):
  a = x1f - x1i
  b = x2i - x2f
  c = y1f - y1i
  d = y2i - y2f

  det = a*d - b*c

  if det == 0:
    # Segments are parallel. Even if they're collinear,
    # this is practically not an intersection anyway, so just
    # return False
    return False, np.inf

  lx = x2i - x1i
  ly = y2i - y1i

  t1 = (d*lx - b*ly)/det
  t2 = (-c*lx + a*ly)/det
  return 0 <= t1 and t1 <= 1 and 0 <= t2, t1

# From https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
def sign(p1, p2, p3):
  return (p1[0] - p3[0]) * (p2[1] - p3[1]) - (p2[0] - p3[0]) * (p1[1] - p3[1])

# From https://stackoverflow.com/questions/2049582/how-to-determine-if-a-point-is-in-a-2d-triangle
def pointInTriangle(pt, v1, v2, v3):
  d1 = sign(pt, v1, v2)
  d2 = sign(pt, v2, v3)
  d3 = sign(pt, v3, v1)

  has_neg = (d1 < 0) or (d2 < 0) or (d3 < 0)
  has_pos = (d1 > 0) or (d2 > 0) or (d3 > 0)

  return not (has_neg and has_pos)

def segment_intersects_triangle(x1i, y1i, x1f, y1f, \
                                xt1, yt1, \
                                xt2, yt2, \
                                xt3, yt3):
  return lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                               xt1, yt1, xt2, yt2) or \
         lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                               xt2, yt2, xt3, yt3) or \
         lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                               xt3, yt3, xt1, yt1) or \
         (pointInTriangle((x1i, y1i), \
                          (xt1, y11), (xt2, yt2), (xt3, yt3)) and \
          pointInTriangle((x1f, y1f), \
                          (xt1, y11), (xt2, yt2), (xt3, yt3)))

def min_dist_btw_line_segments(x1i, y1i, x1f, y1f, \
                               x2i, y2i, x2f, y2f):
  if lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                           x2i, y2i, x2f, y2f):
    return 0.
  return min(dist_point_to_line_segment(x1i, y1i, x1f, y1f, x2i, y2i), \
             dist_point_to_line_segment(x1i, y1i, x1f, y1f, x2f, y2f), \
             dist_point_to_line_segment(x2i, y2i, x2f, y2f, x1i, y1i), \
             dist_point_to_line_segment(x2i, y2i, x2f, y2f, x1f, y1f))

def min_dist_btw_line_segments_3d(s1, f1, \
                                  s2, f2):
  # Assume line segments don't intersect
  return min(dist_point_to_line_segment_3d(s1, f1, s2), \
             dist_point_to_line_segment_3d(s1, f1, f2), \
             dist_point_to_line_segment_3d(s2, f2, s1), \
             dist_point_to_line_segment_3d(s2, f2, f1))

def min_dist_segment_connecting_line_segments(x1i, y1i, x1f, y1f, \
                                              x2i, y2i, x2f, y2f, verbose=False):
  intersect, percent = lineSegmentsIntersectPercent(x1i, y1i, x1f, y1f, \
                                                    x2i, y2i, x2f, y2f, True)
  if intersect:
    pt = (1 - percent)*np.array([x1i, y1i]) + percent*np.array([x1f, y1f])
    return 0., np.concatenate((pt, pt))

  x, y, best_dist = get_closest_point_on_line_segment(x1i, y1i, x1f, y1f, x2i, y2i)
  best_segment = np.array([x, y, x2i, y2i])
  x, y, dist = get_closest_point_on_line_segment(x1i, y1i, x1f, y1f, x2f, y2f)
  if dist < best_dist:
    best_dist = dist
    best_segment = np.array([x, y, x2f, y2f])
  x, y, dist = get_closest_point_on_line_segment(x2i, y2i, x2f, y2f, x1i, y1i)
  if dist < best_dist:
    best_dist = dist
    best_segment = np.array([x, y, x1i, y1i])
  x, y, dist = get_closest_point_on_line_segment(x2i, y2i, x2f, y2f, x1f, y1f)
  if dist < best_dist:
    best_dist = dist
    best_segment = np.array([x, y, x1f, y1f])
  return best_dist, best_segment

def segment_intersects_rect(x1i, y1i, x1f, y1f, \
                            xir, yir, \
                            xfr, yfr):
  return point_in_rect(x1i, y1i, \
                       xir, yir, xfr, yfr) or \
         point_in_rect(x1f, y1f, \
                       xir, yir, xfr, yfr) or \
         lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                               xir, yir, xir, yfr) or \
         lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                               xir, yfr, xfr, yfr) or \
         lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                               xfr, yfr, xfr, yir) or \
         lineSegmentsIntersect(x1i, y1i, x1f, y1f, \
                               xfr, yir, xir, yir)

def closestDistanceBetweenLines(a0,a1,b0,b1,clampAll=False,clampA0=False,clampA1=False,clampB0=False,clampB1=False):

    ''' Given two lines defined by numpy.array pairs (a0,a1,b0,b1)
        Return the closest points on each segment and their distance
    '''

    # If clampAll=True, set all clamps to True
    if clampAll:
        clampA0=True
        clampA1=True
        clampB0=True
        clampB1=True


    # Calculate denomitator
    A = a1 - a0
    B = b1 - b0
    magA = np.linalg.norm(A)
    magB = np.linalg.norm(B)
    
    _A = A / magA
    _B = B / magB
    
    cross = np.cross(_A, _B);
    denom = np.linalg.norm(cross)**2
    
    
    # If lines are parallel (denom=0) test if lines overlap.
    # If they don't overlap then there is a closest point solution.
    # If they do overlap, there are infinite closest positions, but there is a closest distance
    if not denom:
        d0 = np.dot(_A,(b0-a0))
        
        # Overlap only possible with clamping
        if clampA0 or clampA1 or clampB0 or clampB1:
            d1 = np.dot(_A,(b1-a0))
            
            # Is segment B before A?
            if d0 <= 0 >= d1:
                if clampA0 and clampB1:
                    if np.absolute(d0) < np.absolute(d1):
                        return a0,b0,np.linalg.norm(a0-b0)
                    return a0,b1,np.linalg.norm(a0-b1)
                
                
            # Is segment B after A?
            elif d0 >= magA <= d1:
                if clampA1 and clampB0:
                    if np.absolute(d0) < np.absolute(d1):
                        return a1,b0,np.linalg.norm(a1-b0)
                    return a1,b1,np.linalg.norm(a1-b1)
                
                
        # Segments overlap, return distance between parallel segments
        return None,None,np.linalg.norm(((d0*_A)+a0)-b0)
        
    
    
    # Lines criss-cross: Calculate the projected closest points
    t = (b0 - a0);
    detA = np.linalg.det([t, _B, cross])
    detB = np.linalg.det([t, _A, cross])

    t0 = detA/denom;
    t1 = detB/denom;

    pA = a0 + (_A * t0) # Projected closest point on segment A
    pB = b0 + (_B * t1) # Projected closest point on segment B


    # Clamp projections
    if clampA0 or clampA1 or clampB0 or clampB1:
        if clampA0 and t0 < 0:
            pA = a0
        elif clampA1 and t0 > magA:
            pA = a1
        
        if clampB0 and t1 < 0:
            pB = b0
        elif clampB1 and t1 > magB:
            pB = b1
            
        # Clamp projection A
        if (clampA0 and t0 < 0) or (clampA1 and t0 > magA):
            dot = np.dot(_B,(pA-b0))
            if clampB0 and dot < 0:
                dot = 0
            elif clampB1 and dot > magB:
                dot = magB
            pB = b0 + (_B * dot)
    
        # Clamp projection B
        if (clampB0 and t1 < 0) or (clampB1 and t1 > magB):
            dot = np.dot(_A,(pB-a0))
            if clampA0 and dot < 0:
                dot = 0
            elif clampA1 and dot > magA:
                dot = magA
            pA = a0 + (_A * dot)

    
    return pA,pB,np.linalg.norm(pA-pB)

# Returns HREP Ax <= b
def convert_planar_chull_vrep_to_hrep(chull_points):
  A = np.zeros((len(chull_points), 2)) # One row per point
  b = np.zeros(len(chull_points))
  rot90 = np.array([[0., -1.], [1., 0.]])
  chull_points = np.concatenate((chull_points, chull_points[:1]), 0)
  for row, (p1, p2) in enumerate(zip(chull_points[:-1], chull_points[1:])):
    perp = rot90@(p2 - p1)
    largest_proj = 0.
    for p3 in chull_points:
      proj = perp@(p3 - p1)
      if np.abs(proj) > largest_proj:
        largest_proj = proj

    perp = -np.sign(largest_proj)*perp
    A[row] = perp
    b[row] = perp@p1

  return A, b


# From here: https://3dkingdoms.com/weekly/bbox.cpp
def line_segment_in_box(L1, L2, box):
  box_center = np.mean(box, axis=0)
  box_extent = np.diff(box, axis=0).flatten()*0.5
  # Put line in box space
  LB1 = L1 - box_center
  LB2 = L2 - box_center
  
  # Get line midpoint and extent
  LMid = (LB1 + LB2) * 0.5
  L = (LB1 - LMid)
  LExt = np.abs(L)
  
  # Use Separating Axis Test
  # Separation vector from box center to line center is LMid, since the line is in box space
  if ( abs( LMid[0]) > box_extent[0] + LExt[0]):
    return False
  if ( abs( LMid[1]) > box_extent[1] + LExt[1]):
    return False
  if ( abs( LMid[2]) > box_extent[2] + LExt[2]):
    return False

  # Crossproducts of line and each axis
  if ( abs( LMid[1] * L[2] - LMid[2] * L[1])  >  (box_extent[1] * LExt[2] + box_extent[2] * LExt[1]) ):
    return False
  if ( abs( LMid[0] * L[2] - LMid[2] * L[0])  >  (box_extent[0] * LExt[2] + box_extent[2] * LExt[0]) ):
    return False
  if ( abs( LMid[0] * L[1] - LMid[1] * L[0])  >  (box_extent[0] * LExt[1] + box_extent[1] * LExt[0]) ):
    return False
  # No separating axis, the line intersects
  return True

# Adapted from here: https://3dkingdoms.com/weekly/weekly.php?a=3
def GetIntersection(fDst1, fDst2, P1, P2, Hit):
  if ( (fDst1 * fDst2) >= 0.):
    # Both endpoints are on the same side of the plane
    return False
  if ( fDst1 == fDst2):
    # Both endpoints are on the plane
    return False
  Hit[:] = P1 + (P2-P1) * ( -fDst1/(fDst2-fDst1) )
  return True

def InBox(Hit, B1, B2, Axis):
  if ( Axis==1 and Hit[2] > B1[2] and Hit[2] < B2[2] and Hit[1] > B1[1] and Hit[1] < B2[1]):
    return True
  if ( Axis==2 and Hit[2] > B1[2] and Hit[2] < B2[2] and Hit[0] > B1[0] and Hit[0] < B2[0]):
    return True
  if ( Axis==3 and Hit[0] > B1[0] and Hit[0] < B2[0] and Hit[1] > B1[1] and Hit[1] < B2[1]):
    return True
  return False

# returns true if line (L1, L2) intersects with the boundary of the box (B1, B2)
# returns intersection points in Hits. Requirs Hits is initially empty and appends to it
def CheckLineBox(B1, B2, L1, L2, Hits):
  assert(len(Hits) == 0)

  # Segment entirely outside box
  if (L2[0] < B1[0] and L1[0] < B1[0]):
    return 
  if (L2[0] > B2[0] and L1[0] > B2[0]):
    return
  if (L2[1] < B1[1] and L1[1] < B1[1]):
    return
  if (L2[1] > B2[1] and L1[1] > B2[1]):
    return
  if (L2[2] < B1[2] and L1[2] < B1[2]):
    return
  if (L2[2] > B2[2] and L1[2] > B2[2]):
    return

  # Segment entirely inside box
  if (L1[0] > B1[0] and L1[0] < B2[0] and \
      L1[1] > B1[1] and L1[1] < B2[1] and \
      L1[2] > B1[2] and L1[2] < B2[2] and \
      L2[0] > B1[0] and L2[0] < B2[0] and \
      L2[1] > B1[1] and L2[1] < B2[1] and \
      L2[2] > B1[2] and L2[2] < B2[2]):
    return

  Hit = np.zeros(3)
  if (GetIntersection( L1[0]-B1[0], L2[0]-B1[0], L1, L2, Hit) and InBox( Hit, B1, B2, 1 )):
    Hits.append(np.copy(Hit))
  if (GetIntersection( L1[1]-B1[1], L2[1]-B1[1], L1, L2, Hit) and InBox( Hit, B1, B2, 2 )):
    Hits.append(np.copy(Hit))
  if (GetIntersection( L1[2]-B1[2], L2[2]-B1[2], L1, L2, Hit) and InBox( Hit, B1, B2, 3 )):
    Hits.append(np.copy(Hit))
  if (GetIntersection( L1[0]-B2[0], L2[0]-B2[0], L1, L2, Hit) and InBox( Hit, B1, B2, 1 )):
    Hits.append(np.copy(Hit))
  if (GetIntersection( L1[1]-B2[1], L2[1]-B2[1], L1, L2, Hit) and InBox( Hit, B1, B2, 2 )):
    Hits.append(np.copy(Hit))
  if (GetIntersection( L1[2]-B2[2], L2[2]-B2[2], L1, L2, Hit) and InBox( Hit, B1, B2, 3 )):
    Hits.append(np.copy(Hit))

  assert(len(Hits) <= 2)

# https://stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
def line_segment_intersects_sphere(p1, p2, c, r):
  d = p2 - p1
  f = p1 - c
  a = d.dot(d) ;
  b = 2*f.dot(d);
  c = f.dot(f) - r*r

  discriminant = b*b-4*a*c
  if discriminant >= 0:
    # line containing segment didn't totally miss sphere,
    # so there is a solution to
    # the equation.

    discriminant = np.sqrt(discriminant)

    # either solution may be on or off the segment so need to test both
    # t1 is always the smaller value, because BOTH discriminant and
    # a are nonnegative.
    t1 = (-b - discriminant)/(2*a)
    t2 = (-b + discriminant)/(2*a)

    # 3x HIT cases:
    #          -o->             --|-->  |            |  --|->
    # Impale(t1 hit,t2 hit), Poke(t1 hit,t2>1), ExitWound(t1<0, t2 hit),

    # 3x MISS cases:
    #       ->  o                     o ->              | -> |
    # FallShort (t1>1,t2>1), Past (t1<0,t2<0), CompletelyInside(t1<0, t2>1)

    if t1 >= 0 and t1 <= 1:
      # t1 is an intersection
      # Impale or Poke
      return t1

    # here t1 didn't intersect so we either started
    # inside the sphere or are completely past it

    if t2 >= 0 and t2 <= 1:
      # ExitWound
      return t2

    # no intersection: FallShort, Past, CompletelyInside

  return np.inf
