from math import *
from random import *
import numpy

class KalmanFilter:

	collisionUncertainty = 300.

	def __init__(self, worldDimensions=[0,0], uncertainty=0.1):
		width = worldDimensions[0]
		height = worldDimensions[1]

		self.P =  numpy.matrix([[1000., 0., 0., 0.], 
								[0., 1000., 0., 0.], 
								[0., 0., 1000., 0.], 
								[0., 0., 0., 1000.]]) # initial uncertainty

		self.F =  numpy.matrix([[1., 0., 1.0, 0.], 
								[0., 1., 0., 1.0], 
								[0., 0., 1., 0.], 
								[0., 0., 0., 1.]]) # next state function

		self.H =  numpy.matrix([[1., 0., 0., 0.], 
								[0., 1., 0., 0.]]) # measurement function

		self.R =  numpy.matrix([[uncertainty, 0.], 
								[0., uncertainty]]) # measurement uncertainty

		self.I =  numpy.matrix([[1., 0., 0., 0.], 
								[0., 1., 0., 0.], 
								[0., 0., 1., 0.], 
								[0., 0., 0., 1.]]) # identity matrix

		self.x = numpy.matrix([[randint(0, width)], 
								[randint(0, height)], 
								[0.], 
								[0.]]) # initial state (location and velocity)

		self.u = numpy.matrix([[random() * 4 - 2], 
								[random() * 4 - 2], 
								[0.], 
								[0.]]) # external motion

		#self.u = numpy.matrix([[0.], [0.], [0.], [0.]])
		#self.x = numpy.matrix([[0.], [0.], [0.], [0.]])

		self.count = 0

	def clone(self, other):
		self.P = numpy.matrix(other.P)
		self.F = numpy.matrix(other.F)
		self.H = numpy.matrix(other.H)
		self.R = numpy.matrix(other.R)
		self.I = numpy.matrix(other.I)
		self.u = numpy.matrix(other.u)
		self.x = numpy.matrix(other.x)

		self.count = other.count

	def setPosition(self, x, y):
		# Zero out the existing positions
		self.x = numpy.matrix([[0.,0.,0.,0.],
							   [0.,0.,0.,0.],
							   [0.,0.,1.,0.],
							   [0.,0.,0.,1.]]) * self.x
		# Add in the new ones
		self.x = self.x + numpy.matrix([[x],[y],[0.],[0.]])

	def updateFilter(self, measurement):

		# prediction
		self.x = (self.F * self.x) + self.u
 		self.P = self.F * self.P * numpy.transpose(self.F)
        
        # measurement update
		self.Z = numpy.matrix([measurement])
		self.y = numpy.transpose(self.Z) - (self.H * self.x)
		self.S = self.H * self.P * numpy.transpose(self.H) + self.R
		self.K = self.P * numpy.transpose(self.H) * numpy.linalg.inv(self.S)
		self.x = self.x + (self.K * self.y)
		self.P = (self.I - (self.K * self.H)) * self.P

		self.count += 1

	def updateVelocityVectors(self, dx, dy):
		# Zero out the existing velocity vectors
		self.x = numpy.matrix([[1.,0.,0.,0.],
							   [0.,1.,0.,0.],
							   [0.,0.,0.,0.],
							   [0.,0.,0.,0.]]) * self.x
		# Add in the new ones
		self.x = self.x + numpy.matrix([[0.],[0.],[dx],[dy]])
		
		# Add in some uncertainty to the velocity values
		self.P = self.P + numpy.matrix([[0., 0., 0., 0.],
										[0., 0., 0., 0.],
										[0., 0., self.collisionUncertainty, 0.],
										[0., 0., 0., self.collisionUncertainty]])

	def measurement_prob(self, measurement):
		new_x = self.x.tolist()[0][0] + self.x.tolist()[2][0]
		new_y = self.x.tolist()[1][0] + self.x.tolist()[3][0]

		distance = self.distanceBetween([new_x, new_y], measurement)

		if distance == 0.0 :  
			distance = 0.0000000001

		return 1.0 / (distance ** 2)

	def getPrediction(self):
		return self.x

	def getVelocityVectors(self):
		return [self.x.tolist()[2][0], self.x.tolist()[3][0]]

	def distanceBetween(self, point1, point2):
		"""Computes distance between point1 and point2. Points are (x, y) pairs."""
		x1, y1 = point1
		x2, y2 = point2
		return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)