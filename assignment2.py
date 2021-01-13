#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Twist
import tf
from tf import transformations
import math


#Xhorda Cenaj 150170905 
#Medina Zaganjori 150160908
waypoint=None
old_d = 0

#waypoint callback
def waypoint_callback(msg): #  callback

    #***************************************
    #***          Obtain current destination
    #***************************************

    #save waypoint data for printing out in main loop
    global waypoint
    waypoint=msg;


if __name__ == '__main__':

    #setup ROS node, subscribe waypoint_cb to the topic /waypoint_cmd & publish motor commands
    rospy.init_node("crazy_driver_456")
    waypoint_subscriber = rospy.Subscriber("/waypoint_cmd", Transform, waypoint_callback) # <--- set up callback
    motor_command_publisher = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=100)
    #you could in principle also subscribe to the laser scan as is done in assignment 1.

    #setup transform cache manager
    listener = tf.TransformListener()

    #start a loop; one loop per second
    delay = rospy.Rate(1.0); # perhaps this could be faster for a controller?
    while not rospy.is_shutdown():


        #***************************************
        #***          Obtain current robot pose
        #***************************************
        
        try:
            #grab the latest available transform from the odometry frame (robot's original location - usually the same as the map unless the odometry becomes inaccurate) to the robot's base.
            
            (translation,orientation) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0));
        except  (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print("EXCEPTION:",e)
            #if something goes wrong with this just go to bed for a second or so and wake up hopefully refreshed.
            delay.sleep()
            continue
        

        #***************************************
        #***          Print current robot pose
        #***************************************

        #Print out the x,y coordinates of the transform
        print("Robot is believed to be at (x,y): (",translation[0],",",translation[1],")")

        #Convert the quaternion-based orientation of the latest message to Euler representation in order to get z axis rotation
        r_xorient, r_yorient, r_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(orientation))
        robot_theta = r_zorient  # only need the z axis
        print("Robot is believed to have orientation (theta): (",robot_theta,")\n")

        #***************************************
        #***          Print current destination
        #***************************************

        # the waypoint variable is filled in in the waypoint_callback function above, which comes from incoming messages
        # subscribed to in the .Subscriber call above.

        #Print out the x,y coordinates of the latest message
        print("Current waypoint (x,y): (",waypoint.translation.x,",",waypoint.translation.y,")")

        #Convert the quaternion-based orientation of the latest message to angle-axis in order to get the z rotation & print it.
        waypointrotq = [waypoint.rotation.x,waypoint.rotation.y,waypoint.rotation.z,waypoint.rotation.w]
        w_xorient, w_yorient, w_zorient = transformations.euler_from_matrix(transformations.quaternion_matrix(waypointrotq))
        waypoint_theta=w_zorient # only need the z axis
        print("Current waypoint (theta): (",waypoint_theta,")\n")

        #***************************************
        #***          DRIVE THE ROBOT HERE (same as with assignment 1 
        #**           - except you are driving towards a goal not away from an obstacle)
        #***************************************

        #for containing the motor commands to send to the robot
        motor_command=Twist()

	#######################################################################
	# Calculation of cosine and sin of robot state angle or initial orientation

	orientation_sin = math.sin(robot_theta)
	orientation_cos = math.cos(robot_theta)


	# New coordinates of robot's location by using current coordinates 
	coordinate_x = orientation_cos * (waypoint.translation.x - translation[0]) + orientation_sin * (waypoint.translation.y-translation[1])

	coordinate_y = orientation_cos * (waypoint.translation.y - translation[1]) - orientation_sin * (waypoint.translation.x - translation[0])

	# Here we will find the distance between robot's coordinates and waypoint x, y coordinates. In order to 
	# solve the noticeable defects that can be presented from floating point computation we will not use
	# the sqrt function but math.hypot function (Euclidean Distance)
	# We use atan2 function to calculate angle through distance from the robot to goal location by using Moving to a point in 2D plane rules

	d = math.atan2(coordinate_y, coordinate_x)
	r = math.hypot(coordinate_x, coordinate_y)

	linear_value = 0.7
	# PD constants
	c1 = 0.8
	c2 = 0.6
	error_calc = math.cos(abs(d))

	#Activate robot's wheels accordingly by publishing the angles so the robot will start driving 
	motor_command.linear.x = linear_value * r * error_calc

	dif = d - old_d
	motor_command.angular.z = c1 * d + c2 * dif

	# We publish both of the speed calculations to the waypoint topic

	motor_command_publisher.publish(motor_command)

	# Since the performance is done in iterations we save the value of d
	old_d = d

	# All constants and values are chosen based on the efficient and smoothness of robot's movement


        #######################################################################

        delay.sleep()
        # we don't need to call spinOnce like in roscpp as callbacks happen in different threads
    
    
    print("ROS shutdown now I will also go to sleep. I hope I didn't crash. Night night.")
