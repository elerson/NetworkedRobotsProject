
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

class RSSIKalmanFilterRecursive:
	def __init__(self, m, var, measurment_var, d0 = 1.0):

		self.m = np.transpose(m)
		self.P = np.array([[var, 0],[0, var]])
		self.d0 = d0
		self.X = np.array([[0, 0]])
		self.Y = np.array([0])
		self.measurment_var = measurment_var

		self.gamma = self.m
		self.mutex = Lock()

	def addMeasurement(self, distance, measurement):
		if(distance < self.d0):
			return

		y = measurement
		t = -10.0*math.log(distance/self.d0)
		self.mutex.acquire()
		
		self.Y = np.append(self.Y, [y], axis=0)
		self.X = np.append(self.X, [[1, t]], axis=0)
		
		self.mutex.release()

		#print(self.X.shape, self.Y.shape)

	def getResult(self):
		self.mutex.acquire()

		P = pinv(pinv(self.P) + (1.0/self.measurment_var)*dot(np.transpose(self.X[1:]), self.X[1:]))	  
		m = dot(P,((1.0/self.measurment_var)*dot(np.transpose(self.X[1:]), self.Y[1:])+ dot(pinv(self.P),self.m)))

		#self.P = P
		#self.m = m
		#m = dot(pinv(dot(np.transpose(self.X[1:]), self.X[1:])), dot(np.transpose(self.X[1:]), self.Y[1:]))
		self.mutex.release()
		return (m, P)

	def getMetricValue(self, distance):
		if(distance < self.d0):
			return self.m[0]

		m, P = self.getResult()
		t = -10.0*math.log(distance/self.d0)

		#print(m)
		self.gamma = m
		print('gamma', self.gamma)
		return m[0] + t*m[1]


	def getGamma(self):
		return self.gamma[1]

class RSSIKalmanFilter:
	def __init__(self, m, var, measurment_var, log=False, d0 = 1.0):

		self.m = np.transpose(np.matrix(m))
		self.P = np.array([[var, 0],[0, var]])
		self.d0 = d0
		self.X = np.array([[0, 0]])
		self.Y = np.array([0])
		self.measurment_var = measurment_var

		self.gamma = self.m
		self.mutex = Lock()
		self.log   = log
		if(self.log):
			home = os.path.expanduser("~")

			log_dir_ = home + '/NetworkedRobotsProject/RSSLog/'
			folders  = glob.glob(log_dir_+'/*')
			folder   = log_dir_ + '/log' + str(len(folders) + 1)
			os.makedirs(folder)
			self.log_data = folder + '/log.txt'
			self.log_data_file  = open(self.log_data, 'a', 0)
			print(self.log_data)


	def setMeasurmentVar(self, measurment_var):
		self.measurment_var = measurment_var

	def saveLog(self, distance,measurement, measurment_var):
		str_data = str(measurement) + ',' + str(distance) + ',' + str(measurment_var) + ',' + str(self.getGamma()) + '\n'
		print(str_data)
		self.log_data_file.write(str_data)

	def addMeasurement(self, distance, measurement):

		if(distance < self.d0):
			return

		y = measurement
		t = -10.0*math.log(distance/self.d0)
		self.mutex.acquire()
		
		self.Y = np.array([y])
		self.X = np.matrix([[1, t]])
		
		P = pinv(pinv(self.P) + (1.0/self.measurment_var)*dot(np.transpose(self.X), self.X))
		#print(P)
		self.m = dot(P,((1.0/self.measurment_var)*np.transpose(self.X)*y + dot(pinv(self.P),self.m)))
		self.P = P


		#print(self.P,self.m)
		
		self.mutex.release()
		if(self.log):
			self.saveLog(distance, measurement, self.measurment_var)


		#print(self.X.shape, self.Y.shape)

	def getResult(self):
		#self.mutex.acquire()

		#P = pinv(pinv(self.P) + (1.0/self.measurment_var)*dot(np.transpose(self.X[-1]), self.X[-1]))	  
		#m = dot(P,((1.0/self.measurment_var)*dot(np.transpose(self.X[-1]), self.Y[-1])+ dot(pinv(self.P),self.m)))

		#self.P = P
		#self.m = m
		#m = dot(pinv(dot(np.transpose(self.X[1:]), self.X[1:])), dot(np.transpose(self.X[1:]), self.Y[1:]))
		#self.mutex.release()
		return (self.m, self.P)

	def getMetricValue(self, distance):
		if(distance < self.d0):
			return self.m[0][0,0]

		m, P = self.getResult()
		t = -10.0*math.log(distance/self.d0)

		
		self.gamma = m
		measure = m[0] + t*m[1]
		return measure[0, 0]


	def getGamma(self):
		return self.gamma[1]

