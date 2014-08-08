Team Members: 

This project was completed by Jaclyn Adams and John Raffensperger. 

Included Files: 

main.py  				- 	python file that contains common methonds and runs main program
dataExtract.py 			- 	python file used to extract data from videos
ForwardMotionModel.py 	-	python file to handle the forward motion of hexbug 
CollisionMotionModel.py - 	python file to handle when the hexbug hits the wall
CircleKalmanFilter.py 	-  	python file for handling non-straight motion filter
KalmanFilter.py 		- 	python file for handling straight motion filter
Tracker.py 				- 	python file for tracking the hexbug

Included Libraries and Versions:

Python - 2.7
Opencv - 2.4.9


Execution: 

To run this project, execute the following steps: 

1. Copy video file or data file to be used into project directory
2. Command line arguments: 
	-d filename		- use to specify file for data file
	-v filename		- use to specify file for video file
3. To run program, use 'python main.py' with appropriate command line arguments 
	ex. 'python main.py -d centroid_data'
4. After completion, data predictions will be written into predictions.txt
 