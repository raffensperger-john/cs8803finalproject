from math import *
from ForwardMotionModel import *
from CollisionMotionModel import *

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

	#### Constants #####
	numberOfParticles = 100
	# Number of steps to wait before registering consecutive collisions
	collisionBuffer = 10

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
		forward = ForwardMotionModel(self.worldDimensions, self, self.numberOfParticles)
		collision = CollisionMotionModel()

		print 'Training the filters...'
		previousLocation = [0, 0]
		for location in data:
			self.doTrackingUpdate(previousLocation, location, forward, collision)
			previousLocation = location


		print 'Predicting ', numStepsInFuture, ' steps ahead'
		estimatedTrack = [previousLocation]
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
			
			if collisionDetected and self.stepCount - self.lastCollision > self.collisionBuffer:
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
			if self.isOutOfBounds(location):
				#print 'Collision', self.bottomWall, self.topWall, self.rightWall, self.leftWall
				return True

		if self.distanceBetween(previousLocation, location) <= 5:
			return True

		return False

	def isOutOfBounds(self, location):
		return location[1] >= self.bottomWall[1] or \
				location[1] <= self.topWall[1] or \
				location[0] >= self.rightWall[0] or \
				location[0] <= self.leftWall[0]

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
