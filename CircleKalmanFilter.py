from math import *
from random import *
import numpy

class CircleKalmanFilter:

    collisionUncertainty = 300.

    def __init__(self, worldDimensions=[0,0], uncertainty=0.1):
        width = worldDimensions[0]
        height = worldDimensions[1]

        self.H = numpy.matrix([[1.,0.],
                               [0.,1.]])

        self.Q = numpy.matrix([[0.,0.],
                               [0.,0.]]) # movement covariance matrix

        self.R = numpy.matrix([[uncertainty, 0.],
                               [ 0.,uncertainty]]) # measurement covariance matrix

        self.I = numpy.matrix([[1.,0.],
                               [0.,1.]]) # identity matrix

        self.x = numpy.matrix([[0.],
                               [0.]])
        
        self.P = numpy.matrix([[1000.,    0.],
                               [   0., 1000.]])

        self.F = numpy.matrix([[0,0],[0,0]])
        
        self.meas = []
        self.angles = []

        self.count = 0
        self.dx = 1
        self.dy = 1
        self.center = numpy.matrix([[randint(int(width * 0.15), int(width * 0.85))], 
                                    [randint(int(height * 0.15), int(height * 0.85))]])
        self.angle_of_rotation = 0.

    def clone(self, other):
        self.F = numpy.matrix(other.F)
        self.P = numpy.matrix(other.P)
        self.H = numpy.matrix(other.H)
        self.R = numpy.matrix(other.R)
        self.I = numpy.matrix(other.I)
        self.Q = numpy.matrix(other.Q)
        self.x = numpy.matrix(other.x)

        self.count = other.count
        self.dx = other.dx
        self.dy = other.dy
        self.center = other.center
        self.angle_of_rotation = other.angle_of_rotation

    def setPosition(self, x, y):
        self.x = numpy.matrix([[x],[y]])

    def circular_path(self, measurement, center, angle_of_rotation, rotation_matrix):
        new_xy = rotation_matrix * (measurement - center) + center
        return (new_xy.tolist()[0][0], new_xy.tolist()[1][0])

    def updateFilter(self, measurement):

        if self.meas == []:
            self.meas = [measurement, measurement]
            self.angles = [atan2(measurement[1], measurement[0])]
        else:
            self.meas.append(measurement)

        prev_meas = self.meas[len(self.meas) - 2]
        new_angle = atan2(measurement[1] - self.center.tolist()[1][0], measurement[0] - self.center.tolist()[0][0])
        prev_angle = atan2(prev_meas[1] - self.center.tolist()[1][0], prev_meas[0] - self.center.tolist()[0][0])
        if new_angle < 0 and prev_angle > 0:
            new_angle += 2 * pi
        self.angles.append(new_angle - prev_angle)
    
        self.angle_of_rotation = sum(self.angles) / len(self.angles)

        self.F = numpy.matrix([[cos(self.angle_of_rotation),-sin(self.angle_of_rotation)],
                               [sin(self.angle_of_rotation), cos(self.angle_of_rotation)]]) # the matrix of partial derivatives
   
        # measurement update
        z = numpy.matrix([[measurement[0]],
                          [measurement[1]]]) # the measurement we receive

        # if we had a different function to measure than just taking
        # h(x) = x, then this would be y = z - h(x)
        y = z - self.x

        # the rest of the equations are the same as in the regular KF!
        S = self.H * self.P * numpy.transpose(self.H) + self.R
        K = self.P * numpy.transpose(self.H) * numpy.linalg.inv(S)
        self.x = self.x + (K * y)
        self.P = (self.I - K * self.H) * self.P

        # the only real change is in the prediction update equations
        # instead of using a matrix to update the x vector, we use
        # the function f
        next_pt = self.circular_path(numpy.matrix([[measurement[0]],[measurement[1]]]), self.center, self.angle_of_rotation, self.F)
        self.x = numpy.matrix([[next_pt[0]],
                               [next_pt[1]]])

        # this equation is the same as in the regular KF!
        self.P = self.F * self.P * numpy.transpose(self.F) + self.Q
        
    def updateVelocityVectors(self, dx, dy):
        self.center[0] *= dx
        self.center[1] *= dy

        self.meas = []
        self.angles = []
        pass

    def measurement_prob(self, measurement):
        newPoint = self.x
        
        distance = self.distanceBetween(newPoint, measurement)

        if distance == 0.0 :  
            distance = 0.0000000001

        return 1.0 / (distance ** 2)

    def getPrediction(self):
        return numpy.matrix([[self.x.tolist()[0][0]],
                             [self.x.tolist()[1][0]],
                             [0.],
                             [0.]])

    def getVelocityVectors(self):
        return [self.dx, self.dy]

    def distanceBetween(self, point1, point2):
        """Computes distance between point1 and point2. Points are (x, y) pairs."""
        x1, y1 = point1
        x2, y2 = point2
        return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

