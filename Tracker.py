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
		#initialize walls in the center until hexbug is able to locate them 
		self.leftWall = [worldDimensions[0]/2,worldDimensions[1]/2]
		self.rightWall = [worldDimensions[0]/2,worldDimensions[1]/2]
		self.topWall = [worldDimensions[0]/2,worldDimensions[1]/2]
		self.bottomWall = [worldDimensions[0]/2,worldDimensions[1]/2]
		self.worldDimensions = worldDimensions

	"""The main tracking function... Takes the data points and figures out the 
	    world and the motion of the hexbug. """
	def trackRobot(self, data, numStepsInFuture=1):
		#print data 

		#call and set motion models
		forward = ForwardMotionModel(self.worldDimensions, self, self.numberOfParticles)
		collision = CollisionMotionModel()

		# use data read from file to train filters for predictions
		print 'Training the filters...'
		previousLocation = [0, 0]
		for location in data:
			self.doTrackingUpdate(previousLocation, location, forward, collision)
			previousLocation = location

		#predict next steps 
		print 'Predicting ', numStepsInFuture, ' steps ahead'
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

			# true or false to determine if collision update motion model is needed
			collisionDetected = self.didCollide(previousLocation, location, withWalls)
			
			if collisionDetected and self.stepCount - self.lastCollision > self.collisionBuffer:
				#update motion based on collision
				wall = self.getWall(location)
				forward.collide(collision, wall)
				self.lastCollision = self.stepCount
			else : 
				#update motion based on no collision/ forward motion
				forward.update(location)

	#use this method when there is a collision to determine which wall the hexbug hit
	#needed for motion model
	def getWall(self, location):
		
		r = self.distanceBetween(location, self.rightWall)
		l = self.distanceBetween(location, self.leftWall)
		t = self.distanceBetween(location, self.topWall)
		b = self.distanceBetween(location, self.bottomWall)

		wallDistance = [r,l,t,b]
		wall = wallDistance.index(min(wallDistance))

		return wall

	#call to determine if hexbug collided with the wall 
	def didCollide(self, previousLocation, location, withWalls=False):
		
		if withWalls:
			if self.isOutOfBounds(location):
				#print 'Collision', self.bottomWall, self.topWall, self.rightWall, self.leftWall
				return True

		if self.distanceBetween(previousLocation, location) <= 5:
			return True

		return False

	# determine if the robot has gone passed a wall
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

	# method for hexbug to find walls
	# wall values are updated each time the hexbug passes the currently set point, and 
	# solidified when hexbug does not pass that point anymore (because hitting wall)
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

	#returns the coordinates of the walls as the hexbug knows them to be
	def getWallCoordinates(self):
		return (self.leftWall, self.rightWall, self.topWall, self.bottomWall)
