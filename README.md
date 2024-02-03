# Decypher the secret code
For this final project, we were tasked with finding a secret message hidden inside a series of different QR codes inside a ROS environment, using Python.
The environment in question was a simulation of one of the labs at DTU, using Gazebo.

The task involved programming a simulated **Turtlebot** to autonomously manoeuvre around the classroom looking for QR codes that held two pieces of information: 
1. One part of the secret message
2. The coordinates of the next QR code

A map of the classroom was generated beforehand, which was used for localisation. The map lacked the information of where the QR codes were, as well as where obstacles were placed, as these were randomly generated each time loading the simulation. 

The final result can be seen in the video `exampleFinalProject.mkv` (~6 min). 
The simulated robot successfully decyphered the secret message fully autonomously, albeit not in a particularly quick manner. 
