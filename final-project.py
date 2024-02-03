#!/usr/bin/env python
from tf2_geometry_msgs import PoseStamped as PS2
import rospy, tf
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import *
from std_msgs.msg import *
import numpy as np


def patrol(client):        # Function to patrol certain parts of the room to find first two QR codes
  waypoints = [  
    [-4.0, 0.35],
    [5.8, -0.13]
  ] 
  for i in range(len(waypoints)):   
    x = waypoints[i][0]
    y = waypoints[i][1]
    print("Going to position X:{}, Y:{} ".format(x,y))
    goal = goHere(x,y)
    client.send_goal(goal)
    client.wait_for_result()
    time = int(rospy.Time.now().to_sec()) + 23
    print("Scanning for QR codes in this area...")
    while time >= rospy.Time.now().to_sec():
      stopSpin(1)                             # Spin for a full circle, or until QR is found
      getQRstatus 
      if qrStatus == 3: 
        print("Found one QR code")
        wander(1)
        return
    print("Found none in this area, trying the other.")
  print("No QRs found. Starting random wandering...")
         

def wander(slow):                                   # Function to wander around in the map
  twist = Twist()
  if g_range_ahead < 0.8: # Turning
    twist.linear.x = 0.0
    twist.angular.z = 0.3
    if slow:
      twist.linear.x = 0.0
      twist.angular.z = 0.1
  else:                   # Driving forward
    twist.linear.x = 0.3
    twist.angular.z = 0.0
    if slow:
      twist.linear.x = 0.2
      twist.angular.z = 0.0
  cmd_vel_pub.publish(twist)
  rate.sleep()

def stopSpin(spin):                                 # Function to stop wandering and/or spin around
  twist = Twist()
  twist.linear.x = 0.0
  twist.angular.z = 0.0
  if spin:
    twist.angular.z = 0.3
  cmd_vel_pub.publish(twist)
  
def findQrCode():                                   # Function to look for a QR code at current position
  print("Scanning for QR")
  global curPlace
  global read
  global place
  read = 0
  time = int(rospy.Time.now().to_sec()) + 23        # Spin in a full circle
  while time >= rospy.Time.now().to_sec(): 
    stopSpin(1)
    getQRstatus()
    if qrStatus == 3 and len(code) > 45:
      curList = extract_info(code)
      curPlace = curList[5]
      if curPlace == place+1:
        break
  stopSpin(0)
  if curPlace == place+1:
    print("Found QR code {}!!".format(curPlace))
    return True
  else:
    print("Found no QR code")
    return False


def scan_callback(msg):                   # Callback function of the laser scanner subscriber
  global g_range_ahead
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  g_range_ahead = min(tmp)


def qrStatusCallback(msg): # Callback function of the QR-status subscriber, updates the global variable qrStatus
  global qrStatus
  qrStatus = msg.data
    
    
def qrCallback(msg): # Callback function of the QR-code subscriber, updates the global variable code
  global code
  code = msg.data
    
    
def getQRstatus():              # Function to get the QR code message
  global read
  codeSub = rospy.Subscriber("/visp_auto_tracker/code_message", String, qrCallback)
  rospy.wait_for_message("/visp_auto_tracker/status", Int8, 1.0)
  if qrStatus == 3 and read == 0:
    stopSpin(0)
    rospy.wait_for_message("/visp_auto_tracker/code_message", String, 8.0)  #Waits for one single msg, timeout 8 sec
    codeSub.unregister()         # Unregister the subscriber, as only one message is needed
    if len(code) > 45:
      read = 1
  else:
    codeSub.unregister()
  
    
def getQRPose(xPos, yPos):              # Function to get the current QR pose (from read coords)
  qrPose = PS2()
  qrPose.header.frame_id = "qrframe"
  qrPose.pose.position.x = xPos
  qrPose.pose.position.y = yPos
  qrPose.pose.position.z = 0   
  return qrPose


def goHere(xPos, yPos):                  # Function to create a goal for the robot to go to
  goal = MoveBaseGoal()
  goal.target_pose.header.frame_id = 'map'
  goal.target_pose.pose.position.x = xPos
  goal.target_pose.pose.position.y = yPos
  goal.target_pose.pose.position.z = 0
  goal.target_pose.pose.orientation.x = 0.0
  goal.target_pose.pose.orientation.y = 0.0
  goal.target_pose.pose.orientation.z = 0.0
  goal.target_pose.pose.orientation.w = 1.0
  return goal


def extract_info(msg): #Extracts info, returns the next x and y coords, but also extracts other info
  split_frames =  msg.split("\r\n") #Splits the message into the different components, puts into array
  placement = int(split_frames[4].strip("N=")) #Extracts the position of the letter
  letter = split_frames[5].strip("L=") #Extracts the letter itself
  x_next = float(split_frames[2].strip("X_next=")) #Extracts raw value for x,y next
  y_next = float(split_frames[3].strip("Y_next="))
  x = float(split_frames[0].strip("X="))
  y = float(split_frames[1].strip("Y="))
  infoList = [x,y,x_next,y_next,letter,placement]
  return infoList


def getQRmapPose(time):     # Function to get the pose of the QR frame we're looking at in map frame
  listener = tf.TransformListener()
  listener.waitForTransform("map", "qrframe", time, rospy.Duration(3))
  trans, rot = listener.lookupTransform('map', 'qrframe', time)
  qrMapPose = PS2()
  qrMapPose.pose.position.x = trans[0]
  qrMapPose.pose.position.y = trans[1]
  qrMapPose.pose.position.z = trans[2]
  qrMapPose.pose.orientation.x = rot[0]
  qrMapPose.pose.orientation.y = rot[1]
  qrMapPose.pose.orientation.z = rot[2]
  qrMapPose.pose.orientation.w = rot[3]
  return qrMapPose
  

def vectorTransform(map0,map1,qr0,qr1):# Function to find the translation matrix of the hidden frame - see report for formula
  # qrVector: vector between QR1 and QR2, as "told" by each QR
  # mapVector: vector between QR1 and QR2 in the map frame
  mapX = map1[0]-map0[0]
  mapY = map1[1]-map0[1]
  qrX = qr1[0]-qr0[0]
  qrY = qr1[1]-qr0[1]
  mapVector = np.array([mapX,mapY]).reshape(2,1)
  qrVector = np.array([qrX,qrY]).reshape(1,2)
  cosAngle = np.dot(qrVector,mapVector)/(np.linalg.norm(mapVector)*np.linalg.norm(qrVector))
  sinAngle = np.sin(np.arccos(cosAngle))
  rotationMatrix = np.array([cosAngle, -sinAngle, sinAngle, cosAngle]).reshape(2,2)
  qrRotated = np.cross(rotationMatrix, qrVector).reshape(2,1)
  positionVector = np.array([qr1[0],qr1[1]]).reshape(2,1)
  rotatedPoint = np.dot(rotationMatrix,positionVector)
  translation = np.array([map1[0]-rotatedPoint[0],map1[1]-rotatedPoint[1]]).reshape(2,1)
  translationMatrix = np.hstack((rotationMatrix,translation))
  translationMatrix = np.vstack((translationMatrix, np.array([0,0,1]).reshape(1,3)))
  return translationMatrix


def newpos(matrix,x,y):   # Function transforming the QR coordinates to map coordinates
  positionVector = np.array([x,y,1]).reshape(3,1)
  newpos = np.dot(matrix,positionVector)
  return newpos


def findTwo(infoList, secretMessage):     #  Function to find the first two QRs
  global place
  global read
  gotTwoQR = 0
  hiddenCoords = []
  mapCoords = []
  while gotTwoQR < 2:                     # Loop to find the first two QR codes
    while qrStatus != 3:
      wander(1)
      getQRstatus()
    if len(code) > 45:
      curList = extract_info(code)        # Extract all information given from QR
      curPlace = curList[5]
      if place != curPlace:
        rospy.sleep(3)
        now = rospy.Time(0)               # Store the time of when we found the QR
        infoList[curPlace-1] = curList
        secretMessage[curPlace-1] = curList[4]
        tmp = [curList[0], curList[1]]
        hiddenCoords.append(tmp)
        gotTwoQR += 1
        place = curPlace
        qrPose = getQRmapPose(now)
        mapCoords.append(qrPose)
        print("QR {} found!".format(curPlace))
      while qrStatus == 3:
        stopSpin(1)
        getQRstatus()
      read = 0
  return infoList, secretMessage, curList, hiddenCoords, mapCoords


def somewhereClose(goal, client):       # Function to explore close to the given coordinate
  goalList = [[goal.target_pose.pose.position.x + 1.0, goal.target_pose.pose.position.y],
              [goal.target_pose.pose.position.x - 1.0, goal.target_pose.pose.position.y],
              [goal.target_pose.pose.position.x - 0.5, goal.target_pose.pose.position.y],
              [goal.target_pose.pose.position.x + 0.5, goal.target_pose.pose.position.y],
              [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y + 1.0],
              [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y - 1.0],
              [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y + 0.5],
              [goal.target_pose.pose.position.x, goal.target_pose.pose.position.y - 0.5]]
  newgoal = goal 
  for i in range(len(goalList)):
      newgoal.target_pose.pose.position.x = goalList[i][0]
      newgoal.target_pose.pose.position.y = goalList[i][1]
      print("Checking another position...")
      client.send_goal(newgoal)
      client.wait_for_result(rospy.Duration(15))
      if findQrCode(): break
  

def main():         # The main function - where all the magic happens. 
  global read
  global place
  rospy.Subscriber("/visp_auto_tracker/status", Int8, qrStatusCallback)
  client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
  client.wait_for_server()
  read = 0
  infoList = [None, None, None, None, None]       # Has all the information stored from the QR-codes
  secretMessage = [None, None, None, None, None]  # Stores the secret message
  curList = [None, None, None, None, None]        # Has only the info stored from the current QR-code
  patrol(client)      # Patrol a pair of hardcoded positions, where its likely we'll find QR codes
  
  infoList, secretMessage, curList, hiddenCoords, mapCoords = findTwo(infoList, secretMessage) # Find the first two QR codes
  stopSpin(0)                               # Stops the robot
  qrmap1 = [mapCoords[0].pose.position.x, mapCoords[0].pose.position.y] 
  qrmap2 = [mapCoords[1].pose.position.x, mapCoords[1].pose.position.y]
  qr1 = hiddenCoords[0]
  qr2 = hiddenCoords[1]                                       # Store 4 vectors for the two QR codes
  transMatrix = vectorTransform(qrmap1, qrmap2, qr1, qr2)# Finds the transformation matrix between map frame and hidden frame
  nextQR = newpos(transMatrix, infoList[place-1][2], infoList[place-1][3])    # Find the position of the new QR in map frame

  while any(x is None for x in infoList):   # Loop to find the rest of the QR codes
    if place == 5:
      place = 0
    if infoList[place] != None:
      nextQR = newpos(transMatrix, infoList[place][2], infoList[place][3])  # Update next if we've already been there
      place += 1
      continue
    print("Next QR {}, Coords: {}{}".format(place+1,nextQR[0],nextQR[1]))
    goal = goHere(nextQR[0]*0.8, nextQR[1]*0.8)   # Go 80% of the way towards the QR code - hopefully it's readable from here
    client.send_goal(goal)
    client.wait_for_result()
    rate.sleep()
    stopSpin(0)
    if not findQrCode():                          # Spin around and look for QR code
      print("Found no way of reaching goal - trying nearby positions")
      somewhereClose(goal, client)                # Search nearby places

    if len(code) > 45:                            # Check that the code is properly read
      curList = extract_info(code)                # Extract the data from the code
      curPlace = curList[5]
      if curPlace == place+1:
        infoList[place] = curList                 
        secretMessage[place] = curList[4]         # Update the secret message
        nextQR = newpos(transMatrix, infoList[place][2], infoList[place][3])  # Find coordinates for next QR
        place = curPlace
    rate.sleep()
  
  stopSpin(0)
  print("Secret is: {}{}{}{}{}".format(secretMessage[0],secretMessage[1],secretMessage[2],
                                       secretMessage[3],secretMessage[4]))
  rospy.sleep(1)


if __name__=='__main__':
  try:
    g_range_ahead = 1                                       # anything to start - used for wandering
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('detective_burger')
    rate = rospy.Rate(10)
    qrStatus = 0
    read = 0
    place = 0
    curPlace = 0
    code = ""
    main()
  except rospy.ROSInterruptException:
    pass

