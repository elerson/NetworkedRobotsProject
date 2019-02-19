#!/usr/bin/python
from bresenham import bresenham
import cv2
import numpy as np

class Map:
  def __init__(self, map_file):
    self.map = cv2.imread(map_file, 0)
    thresh = 127
    self.im_bw = cv2.threshold(self.map, thresh, 255, cv2.THRESH_BINARY)[1]    
  #
  def inLineOfSight(self, point1, point2, threshold=127, num_values = 10):
    #
    #return True, -1, -1
    points_list = np.array(list(bresenham(point1[1], point1[0], point2[1], point2[0])))
    #
    #print(self.im_bw.shape)
    line_values = self.im_bw[points_list[:,0], points_list[:,1]]
    #print(len(np.where( line_values < threshold)[0]), len(np.where( line_values > threshold)[0]))
    #
    try:
      idx     = np.where(line_values<threshold)[0]
      initial = idx[0]
      final   = idx[-1]
    except:
      initial = -1
      final   = -1
    #
    return not len(np.where( line_values < threshold)[0]) > num_values, initial, final
    #


# map_file='/home/elerson/NetworkedRobotsProject/simulated_experiments/2euclideanexperiment/map/ambiente.png'
# map = Map(map_file)
# map.inLineOfSight((111,400),(450,68))
# map.inLineOfSight((47,400),(600,400))
# map.inLineOfSight((47,400),(439,194))
# map.inLineOfSight((47,400),(380,56))
