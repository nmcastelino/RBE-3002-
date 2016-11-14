#!/usr/bin/env python

import rospy, tf, numpy, math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


#sends msg to twist topic to move robot
def publishTwist(linVel, angVel):
    global pub
    msg = Twist()
    msg.linear.x = linVel   #linear velocity
    msg.angular.z = angVel  #angular vel.
    pub.publish(msg) 		#publish message

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    global pose 
    
    pose = Pose()					#finds current position and orientation
    initx = pose.position.x			#x and y position
    inity = pose.position.y

    init_theta = math.degrees(pose.orientation.z)  #current angle orientation
    diffX = goal.pose.position.x - initx			# how far to go in x pos.
    diffY = goal.pose.position.y - inity  			# how far to go in y pos.
    dist = math.sqrt(diffX * diffX + diffY * diffY) # distance formula
    newDist = int(10 * dist)
    dist = newDist / 10
    theta_t = math.degrees(math.atan2(diffY, diffX))# finds angle that points to desired location
    theta1 = init_theta - theta_t					# difference of angles to rotate

   
    a = goal.pose.orientation.x				#quaternion values from rviz message
    b = goal.pose.orientation.y
    c = goal.pose.orientation.z
    d = goal.pose.orientation.w 
    newY = 2 * (a*b + c*d)					#calculation from quaternion to euler
    newX = (a * a + d * d - b * b - c * c)
    yaw = math.atan2(newY, newX)  			#desired orientation

    theta_final = math.degrees(yaw)			#in degrees
    theta2 = -1 * theta_final
    print "spin!"
    print "theta1 %d" % theta1
    rotate(theta1)							#rotates to point to desired location
    print "move!"
    print "DIst %d" % dist
    driveStraight(0.11, dist)				#drive straight the calculated distance
    print "spin!"
    print "theta2 %d" % theta2
    rotate(theta2)							#rotate to follow orientation of arrow of rviz
    print "done"
    pass


#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():
    #print "drive"
    driveStraight(-0.1, 1)   #step away
    #print "do a barrel r2ll"
    rotate(90)				 #rotate 90
    #print " jump"
    driveStraight(0.2, 1)    #go foward
    #print "dodge"	
    rotate(-90)				#return to original orientation

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):

    lin_vel = .5 * ( u1 + u2)  #wheel vels average out is linear
    ang_vel = (u1 - u2)/ 0.23  # angular calculation

  

    now = rospy.Time.now().secs  #initial time
    while (rospy.Time.now().secs - now <= time and not rospy.is_shutdown()): #runs until desired time has ended
        publishTwist(lin_vel, ang_vel) 	#publish on twist to drive bot
    publishTwist(0, 0) 					#publish on twist to stop bot

#This function accepts a speed and a distance for the robot to move in a straight line
def driveStraight(speed, distance):
    global pose 
    rospy.Duration(0.2, 0)

    initX = pose.position.x  #initial x,y positions
    initY = pose.position.y
    destination = False 	#has not reached the new destination

    while(not destination and not rospy.is_shutdown()): #until it gets there
        print "dist = {}".format(distance)
        currX = pose.position.x 	#current x,y positions
        currY = pose.position.y
        diffX = currX - initX		#find the diffence
        diffY = currY - initY		#next line is dist formula
        currPos = math.sqrt(diffX * diffX + diffY * diffY)
        print "iniX = {}".format(initX)  #print out initial and current distance
        print "iniy = {}".format(initY)
        print "posX = {}".format(currX)
        print "posy = {}".format(currY)
        if (currPos >= distance):   #distance reached
            destination = True		#used to exit while loop
            publishTwist(0, 0)      #stop bot
            
        else:
            publishTwist(speed,0)   #otherwise move bot speed amount of m/s
            rospy.sleep(0.15)

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global odom_list
    global pose
    rospy.Duration(0.2, 0)
    diff = angle - math.degrees(pose.orientation.z) #determines how much it needs to rotate
    err = 100
    if (diff < 0):  #turns cw or ccw depending on how much to rotate
        ang = -0.5  #in rad/sec
    else:
        ang = 0.5

    while ((abs(err) >= 2) and not rospy.is_shutdown()): #if error is not within margin of error
        publishTwist(0, ang) #turn 
        err = angle - math.degrees(pose.orientation.z) # acquire new error
        print "err: %d" % err  #print it
    publishTwist(0, 0)  #stop bot
    rospy.sleep(0.15)
#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):

    global odom_list
    global pose
    
    w = speed / radius #determins ang vel
  
    diff = angle - math.degrees(pose.orientation.z)  #determines how much it needs to rotate

    while ((abs(diff) >= 2) and not rospy.is_shutdown()): #if error is not within margin of error
        publishTwist(speed, w) #turn and drive
        diff = angle - math.degrees(pose.orientation.z) # acquire new error
        print "theta: %d" % math.degrees(pose.orientation.z)  #print it

    publishTwist(0, 0)  #stop bot


#Bumper Event Callback function
def readBumper(msg):
    if (msg.state == 1): # the bumper is pressed
        
        print "Bumper activated" #print it
        publishTwist(0, 0) #stop bot
        executeTrajectory() #execute motion to veer away from cause of bump hit


# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)
def timerCallback(event):
    global pose

    pose = Pose() #gets message from pose topic

    odom_list.waitForTransform('odom','base_footprint', rospy.Time(0), rospy.Duration(1.0)) 
    (position, orientation) = odom_list.lookupTransform('odom','base_footprint', rospy.Time(0)) #finds the position and oriention of two objects relative to each other 
    pose.position.x = position[0] #update pose
    pose.position.y = position[1]
    pose.position.z = position[2]
   
    odomW = orientation			#in quaternion
    q = [odomW[0], odomW[1], odomW[2], odomW[3]] #converts array to row/list
    roll, pitch, yaw = euler_from_quaternion(q)  #converts from quaternion to euler

    pose.orientation.z = yaw  	#sets up pose to have theta orientation we need
    theta = math.degrees(yaw)    #conver tot degrees



# This is the program's main function
if __name__ == '__main__':
    
    rospy.init_node('sample_Lab_2_node_aramirez2')
    #globals used in all 
    global pub
    global pose
    global odom_tf
    global odom_list
       
    pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist, None, queue_size=10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    nav_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, navToPose, queue_size=10) # CAll back to handle rviz 2d nav goal
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener() 
    
    # Use this command to make the program wait for some seconds
    rospy.sleep(rospy.Duration(1, 0))

    print "Starting Lab 2"    

    #make the robot keep doing something...
    odom_list = tf.TransformListener()
    rospy.Timer(rospy.Duration(0.01), timerCallback)
    rospy.sleep(rospy.Duration(0.2, 0))

    while(not rospy.is_shutdown()): #while (1) loop
        pass
        

    # Make the robot do stuff...
    #executeTrajectory()			#testing and debugging 

    #driveStraight(0.1, 0.252)

    #spinWheels(0.2, 0.0, 2)

    #rotate(179)

    #driveArc(1, 0.2, 90)

    print "Lab 2 complete!"

