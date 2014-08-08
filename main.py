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


def distanceBetween(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def calculateError(expected, guess):
	total = 0.
	for i in range(len(expected)):
		total += distanceBetween(expected[i], guess[i])**2
	return sqrt(total)

"""Main Program"""
def main(argv):
	try:
		opts, args = getopt.getopt(argv, "d:v:", ["dataFile=", "videoFile="])
	except getopt.GetoptError:
		print 'main.py [-v (--videoFile) || -d (--dataFile)] <file>'
	
	videoFile = ""
	videoInput = False
	dataFile = ""

	for opt, arg in opts:
		if opt in ("-v", "--videoFile"):
			videoInput = True
			videoFile = arg
		elif opt in ("-d", "--dataFile"):
			videoInput = False
			dataFile = arg

	if videoInput :
		data=[]
		#get the centroid data from video
		data = getData(videoFile,data)
		dataFile = "centroid_data"

	arr = []
	inp = open ("centroid_data", "r")
	all = string.maketrans('','')
	nodigs = all.translate(all, string.digits + '-')

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

	tracker = Tracker([854, 480])
	guess = tracker.trackRobot(testData, testSteps)
	#print "Test Data: ", testData 
	#print "\n"
	#print "Guess next location: ", guess
	#print "\n"
	#print "Actual next location: ", predictData

	print 'Writing results to file \'predictions.txt\'...'
	writeDatatoFile("actual.txt", predictData)
	writeDatatoFile("predictions.txt", guess)

	print 'Error', calculateError(predictData, guess)

	plt.plot(*zip(*data))
	plt.plot(*zip(*predictData))
	plt.plot(*zip(*guess))
	plt.gca().invert_yaxis()
	plt.show()

if __name__ == "__main__":
	main(sys.argv[1:])