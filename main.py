from math import *
from random import *
import numpy
import matplotlib.pyplot as plt
from dataExtract import getData

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

class ForwardMotionModel:

	purgeBadAndAddNew = False

	"""Represents the motion model for when the hexbug is moving forward"""
	def __init__(self, worldDimensions, numberOfParticles=100):
		self.previousParticles = []
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

		resample = self.N
		if self.purgeBadAndAddNew:
			resample = self.N * 0.70
			resample = int (resample)
			self.purgeBadAndAddNew = False

		# Resample the existing particles
		for i in range(resample):
			beta += random() * 2.0 * mw
			while beta > w[index]:
				beta -= w[index]
				index = (index + 1) % self.N
			kf = KalmanFilter()
			kf.clone(self.previousParticles[index])
			p3.append(kf)

		# If some were purged, replace them with new random particles
		for i in range(resample, self.N):
			kf = KalmanFilter(self.worldDimensions)
			kf.setPosition(position[0], position[1])
			p3.append(kf)

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
		return (sum_x / len(self.previousParticles), sum_y / len(self.previousParticles))
		
	def getForwardSpeed(self):
		pass

	def collide(self, collisionModel, wall):
		for particle in self.previousParticles:
			[dx, dy] = particle.getVelocityVectors()
			[dx, dy] = collisionModel.update(dx, dy, wall)
			particle.updateVelocityVectors(dx, dy)
		self.purgeBadAndAddNew = True



class CollisionMotionModel(object):
	def __init__(self):
		super(CollisionMotionModel, self).__init__()

    #motion vector of bot assumed to be reflected by the object with a considerable amount of orientation noise
	def update(self, dx, dy, wall):
		#print dx, dy
		if wall == 0  or wall == 1: #left or right wall 
			dx *= -1
		else : #top or bottom wall 
			dy *= -1

		return [dx, dy]


class Tracker:
	"""This is the class that tracks the hexbug"""
	# Instance Variables
	leftWall   = 0  # pixel representing the x value of the left wall  
	rightWall  = 0  # pixel representing the x value of the right wall
	topWall    = 0  # pixel representing the y value of the top wall
	bottomWall = 0  # pixel representing the y value of the bottom wall
	worldDimensions = []

	lastCollision = 0
	stepCount = 0

	def __init__(self, worldDimensions):
		self.leftWall = [worldDimensions[0]/2,worldDimensions[1]/2]
		self.rightWall = [worldDimensions[0]/2,worldDimensions[1]/2]
		self.topWall = [worldDimensions[0]/2,worldDimensions[1]/2]
		self.bottomWall = [worldDimensions[0]/2,worldDimensions[1]/2]
		self.worldDimensions = worldDimensions

	"""The main tracking function... Takes the data points and figures out the 
	    world and the motion of the hexbug. """
	def trackRobot(self, data, numStepsInFuture=1):
		#print data 
		forward = ForwardMotionModel(self.worldDimensions)
		collision = CollisionMotionModel()

		"""Train the filters"""
		previousLocation = [0, 0]
		for location in data:
			self.doTrackingUpdate(previousLocation, location, forward, collision)
			previousLocation = location


		"""Do the prediction"""
		estimatedTrack = []
		for i in range(numStepsInFuture):
			guess = forward.getNextLocation()
			self.doTrackingUpdate(previousLocation, guess, forward, collision, withWalls=True)
			previousLocation = guess
			estimatedTrack.append(guess)

		return estimatedTrack

	def doTrackingUpdate(self, previousLocation, location, forward, collision, withWalls=False):
		self.stepCount += 1
		if location == [-1, -1]:
			#print 'Bad data'
			pass
		else :	
			if withWalls == False:
				self.updateWallCoordinates(previousLocation, location)

			collisionDetected = self.didCollide(previousLocation, location, withWalls)
			#print 'Collision Detected: ', collisionDetected, location
			
			if collisionDetected and self.stepCount - self.lastCollision > 3:
				wall = self.getWall(location)
				#print "wall " , wall 
				# Avoid repeat collision detections
				forward.collide(collision, wall)
				self.lastCollision = self.stepCount
			else : 
				forward.update(location)

	def getWall(self, location):
		
		r = self.distanceBetween(location, self.rightWall)
		l = self.distanceBetween(location, self.leftWall)
		t = self.distanceBetween(location, self.topWall)
		b = self.distanceBetween(location, self.bottomWall)

		wallDistance = [r,l,t,b]
		wall = wallDistance.index(min(wallDistance))

		return wall


	def didCollide(self, previousLocation, location, withWalls=False):
		
		if withWalls:
			if location[1] >= self.bottomWall[1] or \
				location[1] <= self.topWall[1] or \
				location[0] >= self.rightWall[0] or \
				location[0] <= self.leftWall[0]:
				print 'Collision', self.bottomWall, self.topWall, self.rightWall, self.leftWall
				return True

		if self.distanceBetween(previousLocation, location) <= 5:
			return True

		return False

	def distanceBetween(self, point1, point2):
	    """Computes distance between point1 and point2. Points are (x, y) pairs."""
	    x1, y1 = point1
	    x2, y2 = point2
	    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

	def updateWallCoordinates(self, previousLocation, location):
		if location[0] > self.rightWall[0]:
			self.rightWall[0] = location[0] #location[0]
			#print "right wall", self.rightWall

		if location[0] < self.leftWall[0]:
			self.leftWall[0] = location[0] #location[0]
			#print "left wall", self.leftWall

		if location[1] > self.bottomWall[1]:
			self.bottomWall[1] = location[1] #location[1]
			#print "bottom wall", self.bottomWall

		if location[1] < self.topWall[1]:
			self.topWall[1] = location[1] #location[1]
			#print "top wall", self.topWall

	def getWallCoordinates(self):
		return (self.leftWall, self.rightWall, self.topWall, self.bottomWall)


"""Main Program"""

"""Test Set"""
#data = [[669, 420], [-1, -1], [667, 414],[659, 418],[668, 415],[-1, -1],[667, 412],[-1, -1],[-1, -1],[680, 402],[674, 398],[676, 392],[676, 385],[676, 378],[677, 370],
#		[680, 348],[678, 352],[679, 344],[684, 344],[684, 316],[681, 314],[680, 305],[684, 311],[680, 288],[681, 280],[682, 270],[682, 260],[683, 252],[-1, -1],[683, 232],[681, 222],#
#		[679, 214],[678, 207],[676, 199],[673, 189],[672, 182],[670, 174],[668, 166],[665, 156],[663, 147],[660, 139],[657, 129],[653, 121],[649, 112],[644, 104],[639, 96],[636, 93],
#		[632, 91],[627, 91],[619, 92],[610, 92],[601, 92],[591, 91],[580, 91],[571, 92],[561, 92],[552, 92],[543, 92],[533, 93],[523, 91],[513, 90],[504, 89],[494, 87],
#		[486, 89],[478, 90],[469, 91],[461, 93],[453, 96],[444, 99]]
#testSteps = 1
#testData = data
#predictData = [[435, 103]]

"""Real test data"""
#get the centroid data 
data = getData('hexbug-testing_video.mp4')

# filter out the bad points (really just for the graph)
data = [x for x in data if x != [-1,-1]]

testSteps = 30
testData = data[0:len(data)-testSteps]
predictData = data[len(testData):len(data)]

tracker = Tracker([854, 480])
guess = tracker.trackRobot(testData, testSteps)
print "Test Data: ", testData
print "Guess next location: ", guess
print "Actual next location: ", predictData

plt.plot(*zip(*data))
plt.plot(*zip(*predictData))
plt.plot(*zip(*guess))
plt.gca().invert_yaxis()
plt.show()