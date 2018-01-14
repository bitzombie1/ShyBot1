# ShyBot1
This is ver 1 of Joel Hobbie's robotic sculpture code for face detection and servo control. I used the OpenCV  and realsense libs. 

Each sculpture contains an Up Board with a Ubuntu installation. All Ubuntu systems were cloned and restored using Clonezilla. The admin password for these systems is <old standby> . The face detection source code and compiled binary may be found in the /home/chaz/cvtry1 directory. facedetect_6.cpp being the production code. The Ubuntu system is set to boot up without login as there is no console and the program is executed from a crontab command. The on/off button is configured to start up the Ubuntu system and gracefully shut it down. All the face detect software should be uniform but in some cases may sleep for ten seconds to move actuators to a home position. 
	— sculpture 2 is an exception as the depth reading is capped at 1600 instead of 3500. In future software a command line parameter specifying max depth would be a good idea. 


	Each sculpture also has an Arduino that receives serial commands from the Up Board in order to handle the movement of the actuators and servos. Each sculpture has different Arduino code based on the different motor needs.
