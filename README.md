# T265ControlSystem
This is the main control system used to fly the copter from the position reported by the T265 camera.

Written by Mitchell Cooke (cookem4@mcmaster.ca)

## How to Run
In order to run the project one must first run the camera recording app from the repository T265LiveCamData. This file interfaces with the RealSense SDK and writes the camera data to a single line CSV file. The T265ControlSystem project then reads from this single line CSV file to get the camera's current position and orientation. The reason that position and orientation is recieved in the python project through a CSV file is because the RealSense SDK is only available for C/C++ and the python wrapper does not work on ARM based architecture. In summary, the C program T265LiveCamData reads the position and orientation data from the camera and writes it to a single line CSV file. The python program T265ControlSystem then reads this single line CSV file in order to fly the copter.
 
## Technical Details
The control system of this program is written by Keyvan (current PhD candidate) and Mohammad (graduated). Modifications of filtering have been made to allow the camera to report a smoother position and velocity with less noise while minimizing phase delay below 10ms. The matlab code for analysing and designing filters is in the GitHub repository named ____________. There are also several matlab programs for analysing the position reported by the T265 camera, which are contained in the repository named T255MatlabAnalysis. Each of these scripts explain their purpose in the code and some sample data has been included so that the programs can be tested. These matlab scripts can be helpful for understanding the accuracy and capabilities of the camera.



## Current Troubles


## Future Suggestions
 
 
