from math import *
from random import *
import numpy
from dataExtract import getData

class KalmanFilter(object):

	def __init__(self, worldDimensions, uncertainty=0.1):
		super(KalmanFilter, self).__init__()
		width = worldDimensions[0]
		height = worldDimensions[1]

		self.P =  numpy.matrix([[1000., 0., 0., 0.], [0., 1000., 0., 0.], [0., 0., 1000., 0.], [0., 0., 0., 1000.]]) # initial uncertainty
		self.F =  numpy.matrix([[1., 0., 1.0, 0.], [0., 1., 0., 1.0], [0., 0., 1., 0.], [0., 0., 0., 1.]]) # next state function
		self.H =  numpy.matrix([[1., 0., 0., 0.], [0., 1., 0., 0.]]) # measurement function
		self.R =  numpy.matrix([[uncertainty, 0.], [0., uncertainty]]) # measurement uncertainty
		self.I =  numpy.matrix([[1., 0., 0., 0.], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]]) # identity numpy.matrix

		self.x = numpy.matrix([[randint(0, width)], [randint(0, height)], [0.], [0.]]) # initial state (location and velocity)
		self.u = numpy.matrix([[random() * 4 - 2], [random() * 4 - 2], [0.], [0.]]) # external motion

		self.u = numpy.matrix([[0.], [0.], [0.], [0.]])
		self.x = numpy.matrix([[0.], [0.], [0.], [0.]])

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
        

	def measurement_prob(self, measurement):
		new_x = self.x.tolist()[0][0] + self.x.tolist()[2][0]
		new_y = self.x.tolist()[1][0] + self.x.tolist()[3][0]

		return self.distanceBetween([new_x, new_y], measurement)

	def getPrediction(self):
		return self.x

	def distanceBetween(self, point1, point2):
		"""Computes distance between point1 and point2. Points are (x, y) pairs."""
		x1, y1 = point1
		x2, y2 = point2
		return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

class ForwardMotionModel(object):
	
	previousParticles = []
	N = 0
	worldDimensions = ()

	"""Represents the motion model for when the hexbug is moving forward"""
	def __init__(self, worldDimensions, numberOfParticles=100):
		super(ForwardMotionModel, self).__init__()
		self.N = numberOfParticles
		self.worldDimensions = worldDimensions
	
	def update(self, position):
		# --------
		#
		# Make particles
		# 
		if self.previousParticles == []:
			for i in range(self.N):
				r = KalmanFilter(self.worldDimensions)
				self.previousParticles.append(r)

		newParticles = []

		# --------
		#
		# Update particles
		#     

		# measurement update
		w = []
		for i in range(len(self.previousParticles)):
			w.append(self.previousParticles[i].measurement_prob(position))
			self.previousParticles[i].updateFilter(position)

		# resampling
		p3 = []
		index = int(random() * self.N)
		beta = 0.0
		mw = max(w)
		for i in range(self.N):
			beta += random() * 2.0 * mw
			while beta > w[index]:
				beta -= w[index]
				index = (index + 1) % self.N
			p3.append(self.previousParticles[index])

		self.previousParticles = p3
	    

	def getNextLocation(self):
		sum_x = 0
		sum_y = 0
		for i in range(len(self.previousParticles)):
			p = self.previousParticles[i]
			next_x = p.getPrediction().tolist()[0][0] + p.getPrediction().tolist()[2][0]
			next_y = p.getPrediction().tolist()[1][0] + p.getPrediction().tolist()[3][0]
			sum_x += next_x
			sum_y += next_y
			print p.getPrediction()
		return (sum_x / len(self.previousParticles), sum_y / len(self.previousParticles))
		
	def getForwardSpeed(self):
		pass


class CollisionMotionModel(object):
	def __init__(self):
		super(CollisionMotionModel, self).__init__()

    #motion vector of bot assumed to be reflected by the object with a considerable amount of orientation noise
	def update(self, position):
		pass



class Tracker(object):
	"""This is the class that tracks the hexbug"""
	# Instance Variables
	leftWall   = 0  # pixel representing the x value of the left wall  
	rightWall  = 0  # pixel representing the x value of the right wall
	topWall    = 0  # pixel representing the y value of the top wall
	bottomWall = 0  # pixel representing the y value of the bottom wall
	worldDimensions = []

	def __init__(self, worldDimensions):
		self.leftWall = worldDimensions[0]/2
		self.rightWall = worldDimensions[0]/2
		self.topWall = worldDimensions[1]/2
		self.bottomWall = worldDimensions[1]/2
		self.worldDimensions = worldDimensions

	"""The main tracking function... Takes the data points and figures out the 
	    world and the motion of the hexbug. """
	def trackRobot(self, data):
		print data 
		forward = ForwardMotionModel(self.worldDimensions)
		collision = CollisionMotionModel()

		previousLocation = [0, 0]
		for location in data:
			if location == [-1, -1]:
				print 'Bad data'
				continue
				
			self.updateWallCoordinates(previousLocation, location)

			print 'Collision Detected: ', self.didCollide(previousLocation, location), location
			
			if self.didCollide(previousLocation, location) :
				collision.update(location)
				formard = ForwardMotionModel(self.worldDimensions)
			
			else : 
				forward.update(location)

			previousLocation = location

		return forward.getNextLocation()

	def didCollide(self, previousLocation, location):
		
		#if location[1] >= self.bottomWall or \
		#   location[1] <= self.topWall or \
		#   location[0] >= self.rightWall or \
		#   location[0] <= self.leftWall:
		#   return True

		if self.distanceBetween(previousLocation, location) <= 5:
			return True

		return False

	def distanceBetween(self, point1, point2):
	    """Computes distance between point1 and point2. Points are (x, y) pairs."""
	    x1, y1 = point1
	    x2, y2 = point2
	    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

	def updateWallCoordinates(self, previousLocation, location):
		if previousLocation[0] < location[0]:
			# The bug is moving to the right
			if location[0] > self.rightWall:
				self.rightWall = location[0]

		if previousLocation[0] > location[0]:
			# The bug is moving to the left
			if location[0] < self.leftWall:
				self.leftWall = location[0]

		if previousLocation[1] < location[1]:
			# The bug is moving down
			if location[1] > self.bottomWall:
				self.bottomWall = location[1]

		if previousLocation[1] > location[1]:
			# The bug is moving up
			if location[1] < self.topWall:
				self.topWall = location[1]

	def getWallCoordinates(self):
		return (self.leftWall, self.rightWall, self.topWall, self.bottomWall)


"""Main Program"""
#get the centroid data 
#data = getData('hexbug-testing_video.mp4')

tracker = Tracker([1280, 1024])
guess = tracker.trackRobot([[x * 10,x * 10] for x in range(10)])
print "Guess next location: ", guess
print "Actual next location: ", [292, 418]
