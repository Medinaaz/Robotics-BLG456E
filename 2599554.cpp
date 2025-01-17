/*
 * explorer_node_a1_456.cpp
 *
 * BLG456E Assignment 1 skeleton
 *
 * Instructions: Change the laser_callback function to make the robot explore more intelligently, using its sensory data (the laser range array).
 *
 * Advanced: If you want to make use of the robot's mapping subsystem then you can make use of the map in the mapping_callback function.
 *
 * 
 * Medina Zaganjori  150160908
 * Xhorda Cenaj      150170905
 * 
 */

//Common ROS headers.
#include "ros/ros.h"

//This is needed for the data structure containing the motor command.
#include "geometry_msgs/Twist.h"
//This is needed for the data structure containing the laser scan.
#include "sensor_msgs/LaserScan.h"
//This is needed for the data structure containing the map (which you may not use).
#include "nav_msgs/OccupancyGrid.h"
//This is for easy printing to console.
#include <iostream>

// For information on what publishing and subscribing is in ROS, look up the tutorials.
ros::Publisher motor_command_publisher;
ros::Subscriber laser_subscriber;
ros::Subscriber map_subscriber;

// For information on what a "message" is in ROS, look up the tutorials.
sensor_msgs::LaserScan laser_msg;
nav_msgs::OccupancyGrid map_msg;
geometry_msgs::Twist motor_command;

//The following function is a "callback" function that is called back whenever a new laser scan is available.
//That is, this function will be called for every new laser scan.
//
// --------------------------------------------------------------------------------------------
//CHANGE THIS FUNCTION TO MAKE THE ROBOT EXPLORE INTELLIGENTLY.
// --------------------------------------------------------------------------------------------
//
void laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg){
	
    laser_msg=*msg;
    //data structure containing the command to drive the robot
    std::vector<float> laser_ranges;
    laser_ranges=laser_msg.ranges;
    
    int check_front=laser_ranges.size()/2;  //value for the laser in 90 degree
    int check_leftmost= laser_ranges.size()-1; //value for the laser in 180 degree
    int check_rightmost= 0;                    //0 degree
    int check_bit_right= laser_ranges.size()/4; // 45 degree
    int check_bit_left= 3*laser_ranges.size()/4;  // 135 degree
    
 
	if(laser_ranges[check_front] <= 1.2) { //checking whether there is an obtacle in front
		
		if(laser_ranges[check_leftmost] < laser_ranges[check_rightmost]){// if the left wall is closer, turning right
			
			motor_command.angular.z = -4;
			motor_command.linear.x = -0.1;
			motor_command_publisher.publish(motor_command);
		}
		else {
			
			motor_command.angular.z = +4; //otherwise turning left
			motor_command.linear.x = -0.1;
			motor_command_publisher.publish(motor_command);
		}
	}
	
	else if(laser_ranges[check_leftmost]== laser_ranges[check_rightmost]){// if the robot is in between two objects,
																		// while the distance between the objects is the same
		// robot moves forward
		// because we already have a condition for moving forward 
		motor_command.angular.z= +1.1;
		motor_command.linear.x= 0;
		motor_command_publisher.publish(motor_command);
		
	}
	else if(laser_ranges[check_leftmost] <=1 && laser_ranges[check_rightmost] <=1){
		//robot turning back
			motor_command.angular.z = -8;
			motor_command.linear.x = 0;
			motor_command_publisher.publish(motor_command
	}
	
	//obstacle slightly on the left side (45 degree)
	else if(laser_ranges[check_rightmost] <= 1 || laser_ranges[check_bit_right] <= 1) {
		// sightly the robot turns right
			motor_command.angular.z = -0.5;
			motor_command.linear.x = 0;
			motor_command_publisher.publish(motor_command);
	}
	
	//obstacle slightly on the right side (135 degree)
	else if(laser_ranges[check_leftmost] <= 1 || laser_ranges[check_bit_left] <= 1) {
		// the robot turns slightly to its left
			motor_command.angular.z = +0.5;
			motor_command.linear.x = 0;
			motor_command_publisher.publish(motor_command);
	}	
	
	else {
		//no obstacles in front, robot moves forward
		motor_command.linear.x= +1;
		motor_command.angular.z=0;
		motor_command_publisher.publish(motor_command);
	}
    
    motor_command_publisher.publish(motor_command);

    

    //the laser scan is an array (vector) of distances.
    std::cout<<"Number of points in laser scan is: "<<laser_ranges.size()<<std::endl;
    std::cout<<"The distance to the rightmost scanned point is "<<laser_ranges[0]<<std::endl;
    std::cout<<"The distance to the leftmost scanned point is "<<laser_ranges[laser_ranges.size()-1]<<std::endl;
    std::cout<<"The distance to the middle scanned point is "<<laser_ranges[laser_ranges.size()/2]<<std::endl;

    //You can use basic trignometry with the above scan array and the following information to find out exactly where the laser scan found something:
    std::cout<<"The minimum angle scanned by the laser is "<<laser_msg.angle_min<<std::endl;
    std::cout<<"The maximum angle scanned by the laser is "<<laser_msg.angle_max<<std::endl;
    std::cout<<"The increment in angles scanned by the laser is "<<laser_msg.angle_increment<<std::endl; //should be that angle_min+angle_increment*laser_ranges.size() is about angle_max
    std::cout<<"The minimum range (distance) the laser can perceive is "<<laser_msg.range_min<<std::endl;
    std::cout<<"The maximum range (distance) the laser can perceive is "<<laser_msg.range_max<<std::endl;

}
//
// --------------------------------------------------------------------------------------------
//

//You can also make use of the map which is being built by the "gslam_mapping" subsystem
//There is some code here to help but you can understand the API also
// by looking up the OccupancyGrid message and its members (this is the API for the message).
//If you want me to explain the data structure I will - just ask me in advance of class.
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {

    const bool chatty_map=false;

    map_msg=*msg;

    double map_width=map_msg.info.width;
    double map_height=map_msg.info.width;

    double map_origin_x = map_msg.info.origin.position.x;
    double map_origin_y = map_msg.info.origin.position.y;
    double map_orientation = acos(map_msg.info.origin.orientation.z);

    std::vector<signed char > map = map_msg.data;

    if(chatty_map)std::cout<<"------MAP:------"<<std::endl;
    // Here x and y are incremented by five to make the map fit in the terminal
    // Note that we have lost some map information by shrinking the data
    // this code is mostly to illustrate how the map grid data can be accessed
    for(unsigned int x=0; x<map_width; x+=5) {
        for(unsigned int y=0; y<map_height; y+=5) {

            unsigned int index = x + y*map_width;

            if(map[index] > 50) { // 0 – 100 represents how occupied
                //this square is occupied
                if(chatty_map)std::cout<<"X";
            } else if(map[index]>=0) {
                //this square is unoccupied
                if(chatty_map)std::cout<<" ";
            } else {
                //this square is unknown
                if(chatty_map)std::cout<<"?";
            }
        }
        if(chatty_map)std::cout<<std::endl;
    }
    if(chatty_map)std::cout<<"----------------"<<std::endl;


}

int main(int argc, char **argv)
{
    // must always do this when starting a ROS node - and it should be the first thing to happen
    ros::init(argc, argv, "amble");
    // the NodeHandle object is our access point to ROS
    ros::NodeHandle n;

    // Here we declare that we are going to publish "Twist" messages to the topic /cmd_vel_mux/navi
    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 100);

    // Here we set the function laser_callback to receive new laser messages when they arrive
    laser_subscriber = n.subscribe("/scan", 1000,laser_callback);
    // Here we set the function map_callback to receive new map messages when they arrive from the mapping subsystem
    map_subscriber = n.subscribe("/map", 1000,map_callback);

    //now enter an infinite loop - the callback functions above will be called when new laser or map messages arrive.
    ros::Duration time_between_ros_wakeups(0.001);
    while(ros::ok()) {
        ros::spinOnce();
        time_between_ros_wakeups.sleep();
    }
    return 0;
}
