
class CollisionMotionModel(object):
	def __init__(self):
		super(CollisionMotionModel, self).__init__()

    #motion vector of bot assumed to be reflected by the object with a considerable amount of orientation noise
	def update(self, dx, dy, wall):
		if wall == 0  or wall == 1: #left or right wall 
			dx *= -1
		else : #top or bottom wall 
			dy *= -1

		return [dx, dy]