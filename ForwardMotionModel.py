from math import *
from random import *
import numpy
from CircleKalmanFilter import *
from KalmanFilter import *


class ForwardMotionModel:

	purgeBadAndAddNew = False

	#### Constants ####
	# How many particles are linear kalman filters (vs circular ones)
	percentKalmanFilters = 0.40
	# How many particles to drop on a collision (and replace with random ones)
	percentPurgeOnCollision = 0.80

	"""Represents the motion model for when the hexbug is moving forward"""
	def __init__(self, worldDimensions, tracker, numberOfParticles=100):
		self.previousParticles = []
		self.N = numberOfParticles
		self.worldDimensions = worldDimensions
		self.tracker = tracker
	
	def update(self, position):
		# --------
		#
		# Make particles
		# 
		if self.previousParticles == []:
			for i in range(self.N):
				#r = KalmanFilter(self.worldDimensions)
				#r = CircleKalmanFilter(self.worldDimensions)
				r = self.getRandomFilter()(self.worldDimensions)
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
			resample = self.N * (1-self.percentPurgeOnCollision)
			resample = int (resample)
			self.purgeBadAndAddNew = False

		# Resample the existing particles
		for i in range(resample):
			beta += random() * 2.0 * mw
			while beta > w[index]:
				beta -= w[index]
				index = (index + 1) % self.N
			if isinstance(self.previousParticles[index], KalmanFilter):
				kf = KalmanFilter()
			elif isinstance(self.previousParticles[index], CircleKalmanFilter):
				kf = CircleKalmanFilter()
			kf.clone(self.previousParticles[index])
			#if not self.tracker.isOutOfBounds(self.calculateNextPoint(kf)):
			#	p3.append(kf)
			p3.append(kf)

		# If some were purged, replace them with new random particles
		for i in range(len(p3), self.N):
			#kf = KalmanFilter(self.worldDimensions)
			#kf = CircleKalmanFilter(self.worldDimensions)
			kf = self.getRandomFilter()(self.worldDimensions)
			kf.setPosition(position[0], position[1])
			p3.append(kf)

		self.previousParticles = p3
	    

	def getNextLocation(self):
		sum_x = 0
		sum_y = 0
		for i in range(len(self.previousParticles)):
			(next_x, next_y) = self.calculateNextPoint(self.previousParticles[i])
			sum_x += next_x
			sum_y += next_y
		return (sum_x / len(self.previousParticles), sum_y / len(self.previousParticles))
		
	def calculateNextPoint(self, particle):
		next_x = particle.getPrediction().tolist()[0][0] + particle.getPrediction().tolist()[2][0]
		next_y = particle.getPrediction().tolist()[1][0] + particle.getPrediction().tolist()[3][0]
		return (next_x, next_y)

	def getForwardSpeed(self):
		pass

	def collide(self, collisionModel, wall):
		for particle in self.previousParticles:
			[dx, dy] = particle.getVelocityVectors()
			[dx, dy] = collisionModel.update(dx, dy, wall)
			particle.updateVelocityVectors(dx, dy)
		self.purgeBadAndAddNew = True


	def getRandomFilter(self):
		if random() < self.percentKalmanFilters:
			return KalmanFilter
		else:
			return CircleKalmanFilter
