# Approach based off of technique in paper (http://www.cs.washington.edu/robotics/postscripts/tracking-robocup-04.pdf)
# Utilizes both a Particle Filter and Kalman Filter 

from math import *
import matplotlib.pyplot as plt
from dataExtract import getData
import sys
import getopt
import string

from Tracker import *

#write the predictions into output text file
def writeDatatoFile(filename, data):
	file = open(filename, "w")

	file.write("\n".join(str(i) for i in data))

	file.close()

#calculate distance between points
def distanceBetween(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

#calculate the error betweem points
def calculateError(expected, guess):
	total = 0.
	for i in range(len(expected)):
		total += distanceBetween(expected[i], guess[i])**2
	return sqrt(total)

#run the main program incorporating file reading, tracking, and prediction
"""Main Program"""
def main(argv):
	try:
		opts, args = getopt.getopt(argv, "d:v:", ["dataFile=", "videoFile="])
	except getopt.GetoptError:
		print 'main.py [-v (--videoFile) || -d (--dataFile)] <file>'
	
	videoFile = ""
	videoInput = False
	dataFile = ""

	#get appropriate file to read
	for opt, arg in opts:
		if opt in ("-v", "--videoFile"):
			videoInput = True
			videoFile = arg
		elif opt in ("-d", "--dataFile"):
			videoInput = False
			dataFile = arg

	#get centroid data from video 
	if videoInput :
		data=[]
		#get the centroid data from video
		data = getData(videoFile,data)
		dataFile = "centroid_data"

	arr = []
	inp = open ("centroid_data", "r")
	all = string.maketrans('','')
	nodigs = all.translate(all, string.digits + '-')

	#read data from file
	print 'Reading data from file...'
	for line in inp.readlines():
			arr.append([])
			for i in line.split():
				arr[-1].append(int(i.translate(all, nodigs)))

	data = arr

	print 'Filtering out bad data points...'
	# filter out the bad points (really just for the graph)
	data = [x for x in data if x != [-1,-1]]

	testSteps = 63
	testData = data[0:len(data)-testSteps]
	predictData = data[len(testData):len(data)]

	#create tracker and run tracking/ set prediction results
	tracker = Tracker([854, 480])
	guess = tracker.trackRobot(testData, testSteps)
	
	#write predictions to file 
	print 'Writing results to file \'predictions.txt\'...'
	writeDatatoFile("actual.txt", predictData)
	writeDatatoFile("predictions.txt", guess)

	print 'Error', calculateError(predictData, guess)

	#plot graph 
	plt.plot(*zip(*data))
	plt.plot(*zip(*predictData))
	plt.plot(*zip(*guess))
	plt.gca().invert_yaxis()
	plt.show()

if __name__ == "__main__":
	main(sys.argv[1:])