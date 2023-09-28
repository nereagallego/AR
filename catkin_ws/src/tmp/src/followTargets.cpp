#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <bits/stdc++.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

class FollowTargetsClass {
	ros::NodeHandle nh_;
	ros::Publisher goal_pub_;
	ros::Subscriber position_sub_;
	geometry_msgs::PoseStamped Goal;
        ifstream inFile;
	std::vector<std::vector<float> > targets;
	int currentTarget; //index with the next target to reach


public:
	FollowTargetsClass() { //in the contructor you can read the targets from the text file
	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need



};


int main(int argc, char** argv) {


	ros::init(argc, argv, "followTargets");
	ros::NodeHandle nh("~");
	FollowTargetsClass FT;

	ros::spin();
	return 0;
}

