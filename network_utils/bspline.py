import numpy as np
from math import floor,sqrt
import math



class BSpline:
  def __init__(self, x, y, radius):
    #
    X = []
    Y = []
    self.map_index = {}
    for i in range(1, len(x)):
      distance = self.getDistance(x[i] - x[i-1], y[i] - y[i-1])
      index_begin = len(X)
      print(floor(distance/radius)+1)
      for j in range(int(floor(distance/radius))):
        X.append(x[i-1] + j*(x[i] - x[i-1])/(distance/radius))
        Y.append(y[i-1] + j*(y[i] - y[i-1])/(distance/radius))

      index_end   = len(X)-1
      #
      self.map_index[i] = (index_begin, index_end)
    #
    #
    self.orig_x = x
    self.orig_y = y
    self.x = X
    self.y = Y
    self.s_size = len(X)-3
  #    
  #
  #
  def getDistance(self, x, y):
    return (sqrt(x**2 + y**2))
    #
  def getPos(self, x, t):
    A = np.matrix((1.0/6.0)*np.array([-t**3 + 3*t**2 - 3*t +1, 3*t**3 - 6*t**2 + 4, -3*t**3 + 3*t**2 + 3*t +1, t**3]))
    M = np.matrix(x)
    b = np.dot(A,M.T)
    return b
  #
  def get(self, t):
    #select S
    S = int(floor(t/(1.0/self.s_size)))
    new_t = t/(1.0/self.s_size) - S 
    X = self.getPos(self.x[S:S+4], new_t)
    Y = self.getPos(self.y[S:S+4], new_t)
    #
    return (X[0,0],Y[0,0])
  #
  def getXY(self, T):
    #select S
    X = []
    Y = []
    for t in T:
      xy = self.get(t)
      X.append(xy[0])
      Y.append(xy[1])
    return X,Y
    #
    #
    #
  def getPointsDistance(self, p1, p2):
      x = p2[0] - p1[0]
      y = p2[1] - p1[1]
      return (sqrt(x**2 + y**2))
  #
  def dist_to_segment(self, p, q, r):
    #p->q, r is external
    #
    if(p[0] == q[0] and p[1] == q[1]):
      return float('inf')
    alpha = (r[0]*q[0] - r[0]*p[0] + r[1]*q[1] - r[1]*p[1] - p[0]*(q[0]-p[0]) - p[1]*(q[1] - p[1]))/( (q[1] - p[1])**2 + (q[0]-p[0])**2 )
    #print (alpha)
    if(alpha < 0 or alpha > 1):
      if(alpha < 0):
        return self.getPointsDistance(p, r)
      if(alpha > 1):
        return self.getPointsDistance(q, r)
    else:
      new_p = (p[0] + alpha*(q[0] - p[0]), p[1] + alpha*(q[1] - p[1]))
      #print(new_p)
      return self.getPointsDistance(new_p, r)
      #
  def dist_to_segment_alpha(self, p, q, r):
    #
    alpha = (r[0]*q[0] - r[0]*p[0] + r[1]*q[1] - r[1]*p[1] - p[0]*(q[0]-p[0]) - p[1]*(q[1] - p[1]))/( (q[1] - p[1])**2 + (q[0]-p[0])**2 )
    return alpha
    #

  def getClosestPoint(self, x, y):


    min_dist  = float('inf')
    min_index = -1
    min_t = -1
    r = (x, y)

    for i in range(1, len(self.x)):
      p = (self.x[i-1], self.y[i-1])
      q = (self.x[i], self.y[i])

      #
      dist = self.dist_to_segment(p, q, r)
      t = self.dist_to_segment_alpha(p, q, r)
      if(min_dist > dist):
        min_index = i
        min_dist  = dist
        min_t = t

    p = (self.x[min_index-1], self.y[min_index-1])
    q = (self.x[min_index], self.y[min_index])
    t = self.dist_to_segment_alpha(p, q, r)
    t = max(0, min(1, t))

    x = p[0] + t*(q[0] - p[0])
    y = p[1] + t*(q[1] - p[1])


    s = min_index-2
    #
    closest_t = (1.0/self.s_size)*s + (1.0/self.s_size)*t

    if(closest_t < 0.05):
      return -1, self.get(0.06)

    if(closest_t > 0.95):
      return -1, self.get(0.94)

    return closest_t, (x, y)

  def getDerivative__(self, x, t):
    A = np.matrix((1.0/6.0)*np.array([-3*t**2 + 6*t - 3, 9*t**2 - 12*t , -9*t**2 + 6*t + 3, 3*t**2]))
    M = np.matrix(x)
    b = np.dot(A,M.T)
    return b
  #
  def getDerivative(self, t):
    #select S
    S = int(floor(t/(1.0/self.s_size)))
    new_t = t/(1.0/self.s_size) - S 
    #print('new_t', new_t)
    X = self.getDerivative__(self.x[S:S+4], new_t)
    Y = self.getDerivative__(self.y[S:S+4], new_t)
    #
    return (X[0,0],Y[0,0])
  #




# s = BSpline([0,3,2,8],[0,0,5,5], 0.5)
# #print(s.getClosestPoint(-1,-1))
# T = np.linspace(0, 0.99, 50)
# for t in T:
#   d = s.getDerivative(t)
#   print(math.atan2(d[1], d[0])*180.0/math.pi)

# #result = s.getClosestPoint(1.6,-1)
#print(s.get(result[0]))
#T = np.linspace(0, 0.99, 50)
#X, Y = s.getXY(T)


    