#!/usr/bin/env python

import rospy, tf, math, numpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
# Add additional imports for each of the message types used

wheel_base = 0.2286 #m
wheel_diam = 0.07 #m

global callback_flag

#drive to a goal subscribed as /move_base_simple/goal
def navToPose(goal):
    #take goal params and find delta to currpos params
    print "spin!"
    #turn delta theta using spinWheels(vr, -vr, time to rotate)

    print "move!"
    #go delta x, y using spingWheels(vr, vl, time to dest)
    
    print "spin!"
	#spin delta theta using spingWheels(vr, -vr, time to dest)
    
    print "done"
	#cry
    pass

#Odom "Callback" function.
def readOdom(msg):
    global pose
    #global odom_tf
    #global xPosition
    #global yPosition
    #global theta
    #global odom_list

    pose = msg.pose
    #geo_quat = pose.pose.orientation
    #odom_tf.sendTransform((pose.pose.position.x, pose.pose.position.y, 0),
    #    (pose.pose.orientation.x, pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w),
    #    rospy.Time.now(),
    #    "base_footprint", "odom")

#This function sequentially calls methods to perform a trajectory.
def executeTrajectory():

    driveStraight(0.3, 0.06)
    rotate(1.6)
    driveStraight(0.3, 0.045)
    rotate(-2.4)

#This function accepts two wheel velocities and a time interval.
def spinWheels(u1, u2, time):
    # compute wheel speeds
    w = (u1 - u2) / wheel_base
    u = (wheel_diam / 2) * (u1 + u2)
    start = rospy.Time().now().secs

    while ((rospy.Time().now().secs - start) < time):
        publishTwist(u, w)

    publishTwist(0, 0)

#This function accepts a speed and a distance for the robot to move in a straight line

def driveStraight(speed, distance):

    global pose

    xnaught = pose.pose.position.x #set an origin at the robot's current position
    currpos = Twist()
    objectiveReached = False
    print "values set"
    while (not objectiveReached):

        x = pose.pose.position.x
        dx = x-xnaught
        print "in while loop"
        if (dx >= distance):
            objectiveReached = True
            publishTwist(0, 0)
            print "fuken dun m8"

        else:

            #currpos.linear.x = speed
            #pub.publish(currpos)
            publishTwist(speed, 0)
            print "moving"
            #rospy.sleep(rospy.Duration(0, 0.02))
        #rospy.sleep(rospy.Duration(0, 500000))


#Accepts an angle and makes the robot rotate around it.
"""def rotate(angle):

    global odom_list
    global pose

    transformer = tf.TransformerROS()   
    rotation = numpy.array([[math.cos(angle), -math.sin(angle), 0], #Create th goal rotation
                            [math.sin(angle), math.cos(angle), 0],
                            [0,          0,          1]])

 

#Get all the transforms for frames
    odom_list.waitForTransform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(4.0))
    (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
    T_o_t = transformer.fromTranslationRotation(trans, rot)
    R_o_t = T_o_t[0:3,0:3]

 #Setup goal matrix
    goal_rot = numpy.dot(rotation, R_o_t)
    goal_o = numpy.array([[goal_rot[0,0], goal_rot[0,1], goal_rot[0,2], T_o_t[0,3]],
                 [goal_rot[1,0], goal_rot[1,1], goal_rot[1,2], T_o_t[1,3]],
                 [goal_rot[2,0], goal_rot[2,1], goal_rot[2,2], T_o_t[2,3]],
                 [0,             0,             0,             1]])

    #This code continuously creates and matches coordinate transforms.
    done = False
    while (not done and not rospy.is_shutdown()):
        (trans, rot) = odom_list.lookupTransform('odom', 'base_footprint', rospy.Time(0))
        state = transformer.fromTranslationRotation(trans, rot)
        within_tolerance = abs((state - goal_o)) < .2
        if ( within_tolerance.all() ):
            spinWheels(0,0,0)
            done = True
        else:
            if (angle > 0):
                spinWheels(.1,-.1,.1)
            else:
                spinWheels(-.1,.1,.1)"""

#Accepts an angle and makes the robot rotate around it.
def rotate(angle):
    global pose

    rob_pos = Twist()
    #Had to separate out the quaternion because it was geometry_msgs, not transform
    quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
    initeuler = tf.transformations.euler_from_quaternion(quaternion)
    inityaw = initeuler[2]
    print inityaw
    print angle

    there = False
    while(not there):
        currquaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w)
        curreuler = tf.transformations.euler_from_quaternion(currquaternion)
        curryaw = curreuler[2]
        newang = curryaw - inityaw
        print str(newang) +','+ str(angle)
        if(angle >= 0):
            if(newang >= angle):
                there = True
                rob_pos.angular.z = 0
                pub.publish(rob_pos)
            else:
                rob_pos.angular.z = .75
                pub.publish(rob_pos)
        else:
            if(newang >= abs(angle)):
                there = True
                rob_pos.angular.z = 0
                pub.publish(rob_pos)
            else:
                rob_pos.angular.z = -.75
                pub.publish(rob_pos)

    print "done angle"

#This function works the same as rotate how ever it does not publish linear velocities.
def driveArc(radius, speed, angle):
    pass  # Delete this 'pass' once implemented

#This function takes angular and linear speeds and publishes them to a twist-type message
def publishTwist(linearvalue, angularvalue):

    global pub

    twist = Twist()
    twist.linear.x = linearvalue
    twist.angular.z = angularvalue

    pub.publish(twist)




#Bumper Event Callback function
def readBumper(msg):
    global bumper
    if (msg.state == 1):
        print "boop"
        bumper = 1



# (Optional) If you need something to happen repeatedly at a fixed interval, write the code here.
# Start the timer with the following line of code: 
#   rospy.Timer(rospy.Duration(.01), timerCallback)

def timerCallback(event):
    global pose
    pose = Pose()

    (position, orientation) = odom_list.lookupTransform('...','...', rospy.Time(0)) #finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
	
    pass # Delete this 'pass' once implemented




# This is the program's main function
if __name__ == '__main__':

    # These are global variables. Write "global <variable_name>" in any other function to gain access to these global variables 
    global pub
    global pose
    global odom_tf
    global odom_list
    global bumper

    bumper = 0



    # Change this node name to include your username
    rospy.init_node('lab2')

    # Replace the elipses '...' in the following lines to set up the publishers and subscribers the lab requires
    pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, None, queue_size = 10) # Publisher for commanding robot motion
    bumper_sub = rospy.Subscriber('mobile_base/events/bumper', BumperEvent, readBumper, queue_size=1) # Callback function to handle bumper events
    sub = rospy.Subscriber('/odom', Odometry, readOdom)
    # Use this object to get the robot's Odometry 
    odom_list = tf.TransformListener()
    #odom_tf = tf.TransformBroadcaster()
    #odom_tf.sendTransform((0, 0, 0),(0, 0, 0, 1),rospy.Time.now(),"base_footprint","odom")


    # Use this command to make the program wait for some seconds
    
    rospy.sleep(rospy.Duration(1, 0))
    print "Starting Lab 2"

    while (bumper == 0):
        print bumper

    executeTrajectory()
        #rospy.sleep(0.5)
    #make the robot keep doing something...
    #rospy.Timer(rospy.Duration(1), timerCallback)


    #spinWheels(0.2, 0.5, 5)
    #driveStraight(.3, 0.25)
    #rotate(-0.5)
    #executeTrajectory()
    #readBumper()

    # Make the robot do stuff...
    print "Lab 2 complete!"

