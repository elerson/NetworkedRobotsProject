#!/usr/bin/python
##
## Here we consider the function PL(d) = PL(d0) + 10*n*log(d/d0) -> y = theta0 + theta1*(10*log(d/d0))
## and we want to estimate the PL(d0) a constant and "n"
##
import csv
import numpy as np
from numpy.linalg import pinv
from numpy import dot
import math
from threading import Lock

class RSSIKalmanFilter:
	def __init__(self, m, var, measurment_var, d0 = 1.0):

		self.m = np.transpose(m)
		self.P = np.array([[var[0], 0],[0, var[1]]])
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
		t = 10.0*math.log(distance/self.d0)
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
		t = 10.0*math.log(distance/self.d0)

		#print(m)
		self.gamma = m
		return m[0] + t*m[1]


	def getGamma(self):
		return self.gamma[1]

class RSSIKalmanFilterRecursive:
	def __init__(self, m, var, measurment_var, d0 = 1.0):

		self.m = np.transpose(np.matrix(m))
		self.P = np.array([[var[0], 0],[0, var[1]]])
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
		t = 10.0*math.log(distance/self.d0)
		self.mutex.acquire()
		
		self.Y = np.array([y])
		self.X = np.matrix([[1, t]])
		
		P = pinv(pinv(self.P) + (1.0/self.measurment_var)*dot(np.transpose(self.X), self.X))
		#print(P)
		self.m = dot(P,((1.0/self.measurment_var)*np.transpose(self.X)*y + dot(pinv(self.P),self.m)))
		self.P = P

		#print(self.m)

		#print(self.P,self.m)
		
		self.mutex.release()

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
			return self.m[0]

		m, P = self.getResult()
		t = 10.0*math.log(distance/self.d0)

		#print(m)
		self.gamma = m
		return m[0] - t*m[1]


	def getGamma(self):
		return self.gamma[1]


class MM:
	def __init__(self, m, alpha, d0 = 1.0):

		self.m = m
		self.alpha = alpha
		self.d0 = d0


	def addMeasurement(self, distance, measurement):
		if(distance < self.d0):
			return

		y = measurement - self.m[0]
		t = 10.0*math.log(distance/self.d0)
		self.m[1] = self.alpha*(y/t)	+ (1.0-self.alpha)*self.m[1]

		#print(self.X.shape, self.Y.shape)

	def getResult(self):
		#self.mutex.acquire()

		#P = pinv(pinv(self.P) + (1.0/self.measurment_var)*dot(np.transpose(self.X[-1]), self.X[-1]))	  
		#m = dot(P,((1.0/self.measurment_var)*dot(np.transpose(self.X[-1]), self.Y[-1])+ dot(pinv(self.P),self.m)))

		#self.P = P
		#self.m = m
		#m = dot(pinv(dot(np.transpose(self.X[1:]), self.X[1:])), dot(np.transpose(self.X[1:]), self.Y[1:]))
		#self.mutex.release()
		return self.m




def main():
	kalman = MM([32, 2.0], 0.2, 3.0)#RSSIKalmanFilterRecursive([32.0, 2.0], [.2, 10.0], 100.0, 3.0)
	#kalman = RSSIKalmanFilter([32.0, 2.0], [.2, 10.0], 100.0, 3.0)
	#kalman = RSSIKalmanFilterRecursive([32.0, 2.0], [.2, 10.0], 100.0, 3.0)

	data = []
	with open('data.csv', 'rb') as file:
		reader = csv.reader(file)
		for row in reader:
			data.append([float(row[0]), float(row[1]), float(row[2]), float(row[3])])
	out_data = []
	j = 0
	with open('rss.csv', 'wb') as csvfile:
		wr = csv.writer(csvfile)
		for i in range(2000, len(data)):
			#print(data[i][3])
			kalman.addMeasurement(data[i][0]+2.0, -data[i][3])
			if(j > 10):
				#wr.writerow((j, np.asscalar(kalman.getResult()[0][1])))
				#print( np.asscalar(kalman.getResult()[0][1]))
				wr.writerow((j, kalman.getResult()[1]))
				#wr.w
				#print(kalman.getResult()[0][1])
				#out_data.append(np.asscalar(kalman.getResult()[0][1]))
				#print()
				
			j = j + 1
			#print data
	#print(out_data)

if __name__ == '__main__':              # if we're running file directly and not importing it
    main()                              # run the main function