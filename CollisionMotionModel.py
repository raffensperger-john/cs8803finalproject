#Motion model for detecting collisions of the bot with the wall
#
#This model simply takes in the motion vector of the hexbug and reflects it, for the hexbug bouncing off of the wall

class CollisionMotionModel(object):
	def __init__(self):
		super(CollisionMotionModel, self).__init__()

	def update(self, dx, dy, wall):
		if wall == 0  or wall == 1: #if hexbug hits the left or right wall, reflect the motion along the x axis
			dx *= -1
		else : #if hexbug hits the top or bottom wall, reflect the motion along the y axis 
			dy *= -1

		return [dx, dy]