from math import *

class ForwardMotionModel(object):
	"""Represents the motion model for when the hexbug is moving forward"""
	def __init__(self):
		super(ForwardMotionModel, self).__init__()
	
	def update(self, position):
		pass

	def getNextLocation(self):
		pass

	def getForwardSpeed(self):
		pass


class CollisionMotionModel(object):
	def __init__(self):
		super(CollisionMotionModel, self).__init__()

	def update(self, position):
		pass



class Tracker(object):
	"""This is the class that tracks the hexbug"""
	# Instance Variables
	leftWall   = 0  # pixel representing the x value of the left wall  
	rightWall  = 0  # pixel representing the x value of the right wall
	topWall    = 0  # pixel representing the y value of the top wall
	bottomWall = 0  # pixel representing the y value of the bottom wall

	def __init__(self, worldDimensions):
		self.leftWall = worldDimensions[0]/2
		self.rightWall = worldDimensions[0]/2
		self.topWall = worldDimensions[1]/2
		self.bottomWall = worldDimensions[1]/2

	"""The main tracking function... Takes the data points and figures out the 
	    world and the motion of the hexbug. """
	def trackRobot(self, data):
		forward = ForwardMotionModel()
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
			
			else : 
				forward.update(location)

			previousLocation = location

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
tracker = Tracker([1280, 1024])
tracker.trackRobot([[665, 156],
					[663, 147],
					[660, 139],
					[657, 129],
					[653, 121],
					[649, 112],
					[644, 104],
					[639, 96],
					[636, 93],
					[632, 91],
					[627, 91],
					[619, 92],
					[610, 92],
					[601, 92],
					[591, 91],
					[580, 91],
					[571, 92],
					[561, 92],
					[552, 92],
					[543, 92],
					[533, 93],
					[523, 91],
					[513, 90],
					[504, 89],
					[494, 87],
					[486, 89],
					[478, 90],
					[469, 91],
					[461, 93],
					[453, 96],
					[444, 99],
					[435, 103],])

print tracker.getWallCoordinates()