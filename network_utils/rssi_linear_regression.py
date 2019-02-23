#!/usr/bin/python
##
## Here we consider the function PL(d) = PL(d0) + 10*n*log(d/d0) -> y = theta0 + theta1*(10*log(d/d0))
## and we want to estimate the PL(d0) a constant and "n"
##

import numpy as np
from numpy.linalg import pinv
from numpy import dot
import math
from threading import Lock
import os
import glob
import pickle
import time

class LinearRegressionRSSI:
  def __init__(self, id_, m, var, d0 = 1.0, log = False):
    #
    self.m_prior = np.array(m)
    self.P_prior = np.array([[var, 0],[0, var]])
    self.d0 = d0
    self.X = np.array([[0, 0]])
    self.measurment_var  = np.array([0])
    self.Y = np.array([0])
    self.m = self.m_prior
    #
    self.gamma = self.m_prior[1]
    self.mutex = Lock()

    self.log = log
    self.time = time.time()
    self.max_time = 20.0
    if(self.log):
      home = os.path.expanduser("~")

      log_dir_ = home + '/NetworkedRobotsProject/RSSLog/'
      folders  = glob.glob(log_dir_+'/*')
      folder   = log_dir_ + '/log' + str(len(folders) + 1)
      os.makedirs(folder)
      self.log_data = folder + '/log.pkl'
      self.log_data_file  = open(self.log_data, 'wb')
      print(self.log_data)


  def saveLog(self):
    pickle.dump(self, self.log_data_file, pickle.HIGHEST_PROTOCOL)


    #
  def addMeasurement(self, distance, measurement, variance):
    if(distance < self.d0):
      return
    #
    y = measurement
    t = -10.0*math.log10(distance/self.d0)
    self.mutex.acquire()
    # #
    self.Y = np.append(self.Y, [y], axis=0)
    self.X = np.append(self.X, [[1, t]], axis=0)
    self.measurment_var = np.append(self.measurment_var, [variance], axis=0)
    
    self.mutex.release()

    if (self.log and (time.time() - self.time) > self.max_time ):
      self.time = time.time()
      self.saveLog()

    #
    #print(self.X.shape, self.Y.shape)
    #
  def getResult(self):
    #return [-40, 3], 0
    
    self.mutex.acquire()
    #
    #print(self.measurment_var[1:].shape)
    weight =  np.eye(self.measurment_var[1:].shape[0])
    n = self.measurment_var[1:].shape[0]
    weight[range(n), range(n)] = self.measurment_var[1:]
    #mat[range(n), range(n)]
    #weight = np.matrix()
    #print(weight.shape)
    #dot(np.transpose(self.X[1:]), dot( weight, self.X[1:]))
    #dot(weight, dot(np.transpose(self.X[1:]), self.X[1:]))
    P = pinv(pinv(self.P_prior) + dot(np.transpose(self.X[1:]), dot( weight, self.X[1:])))    
    m = dot(P, dot(self.X[1:].T, dot(weight,self.Y[1:])) + dot(pinv(self.P_prior), self.m_prior))
    #
    #P = pinv(pinv(self.P_prior) + dot(np.transpose(self.X[1:]),self.X[1:]))   
    #m = dot(P, dot(self.X[1:].T, self.Y[1:])) + dot(pinv(self.P_prior), self.m_prior)
    #
    #
    self.mutex.release()
    #print(m, self.X.shape)
    return (m, P)
    #
  def getMetricValue(self, distance):
    if(distance < self.d0):
      return self.m[0]
    #
    m, P = self.getResult()
    t = -10.0*math.log10(distance/self.d0)
    #
    #print(m)
    self.gamma = m
    #print('gamma', self.gamma)
    return m[0] + t*m[1]
    #
    #
  def getMetricValueInSight(self, distance, d_sight, beta):
    if(distance < self.d0):
      return self.m[0]
    #
    m, P = self.getResult()
    t = -10.0*math.log10(distance/self.d0)
    self.m = m
    #
    #print(m)
    self.gamma = m
    #print('gamma', self.gamma)
    return m[0] + t*(m[1]*((distance/d_sight)**beta))
    #
    #
  def getGamma(self):
    return self.gamma[1]


def generate_data(d, PL0, alpha, sigma):
   return -PL0 - 10*alpha*math.log10(d) + np.random.normal(0,math.sqrt(sigma),1)[0]


linear_reg = LinearRegressionRSSI(0, [-40, 4], 0.1)
distances = np.random.uniform(2, 10, 1000)
for d in distances:
  var_ = np.random.uniform(0.5, 100)
  rssi = generate_data(d, 40, 3, var_)
  linear_reg.addMeasurement(d, rssi, var_)


linear_reg.getResult()