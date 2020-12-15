
#include <chrono>
#include <iostream>
#include <math.h>
#include <random>
#include <string>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "voronoi_alg/RobotPos.h"
#include "nav_msgs/Odometry.h"

std::vector<std::string> split_string_by_spaces(std::string inputstr)
{
    std::stringstream tmpss(inputstr);
    std::vector<std::string> out;
    std::string tmps;
    while (getline(tmpss, tmps, ' ')) {
        out.push_back(tmps);
    }
    return out;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  std::string nodenames;
  bool retsucceeded = n.getParam("robot_names_set", nodenames);
	std::vector<std::string> nodenameVec = split_string_by_spaces(nodenames);
	std::vector<ros::Publisher> publist;
	for (size_t i = 0; i < nodenameVec.size(); i++) {
		//publist.push_back(n.advertise<geometry_msgs::Twist>(nodenameVec.at(i)+"/odom", 0));
    publist.push_back(n.advertise<nav_msgs::Odometry>(nodenameVec.at(i)+"/odom", 0));
	}
	//geometry_msgs::Twist msg, msg2, msg3;
  nav_msgs::Odometry msg, msg2, msg3;
	while (ros::ok()){/*
    msg.linear.x = 0.5;
		msg.linear.y = 0.2;
    msg2.linear.x = 0.7;
		msg2.linear.y = 0.5;
    msg3.linear.x = 0.65;
		msg3.linear.y = 0.55;*/
    msg.pose.pose.position.x = 0.2;
		msg.pose.pose.position.y = 0.5;
    msg2.pose.pose.position.x = 0.7;
		msg2.pose.pose.position.y = 0.5;
    msg3.pose.pose.position.x = 0.65;
		msg3.pose.pose.position.y = 0.55;
		publist.at(0).publish(msg);
		publist.at(1).publish(msg2);
		publist.at(2).publish(msg3);
	}
    //std::cout << nodenames.size() << '\n';
    //std::cout << nodenames << '\n';
    //std::cout << "retsucceeded: " << retsucceeded << '\n';
    //ROS_INFO(std::to_string(nodenames.size()).c_str());
    /*for (size_t i = 0; i < nodenames.size(); i++) {
    std::cout << nodenames.at(i) << '\n';
    //ROS_INFO(nodenames.at(i).c_str());
  }*/
    //Voronoicbc instance(n);

    return 0;
}
