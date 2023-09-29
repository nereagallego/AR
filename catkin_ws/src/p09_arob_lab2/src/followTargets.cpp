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
		//open the file
		inFile.open("/home/nerea/AR/catkin_ws/src/p09_arob_lab2/src/targets.txt");
		if (!inFile) {
			cout << "Unable to open file";
			exit(1); // terminate with error
		}
		cout << "llega aqui" << endl;
		//read the targets from the file
		float x, y;

		// while (inFile >> x >> y) {
		// 	std::vector<float> target;
		// 	target.push_back(x);
		// 	target.push_back(y);
		// 	targets.push_back(target);
		// }

		string tmp;
		vector<string>* p = NULL;
		while (!inFile.eof()) {
			std::vector<float> target;
			getline(inFile, tmp, ';');



			x = stof(tmp);
			getline(inFile, tmp, ';');
			y = stof(tmp);

			target.push_back(x);
			target.push_back(y);
			targets.push_back(target);
			cout << x << " " << y << endl;
		}

		
		//close the file
		inFile.close();
		//initialize the current target
		currentTarget = 0;
		//initialize the goal
		Goal.pose.position.x = targets[currentTarget][0];
		Goal.pose.position.y = targets[currentTarget][1];
		cout << "llega aqui" << endl;
		//initialize the publisher
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("goal", 1);
		cout << "llega aqui" << endl;
		//initialize the subscriber
		position_sub_ = nh_.subscribe("/base_pose_ground_truth", 1, &FollowTargetsClass::positionCb, this);
		cout << "llega aqui" << endl;
		
		//publish the goal
		goal_pub_.publish(Goal);
		currentTarget ++;

	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need

	void positionCb(const nav_msgs::Odometry& msg) {
		//check if the robot has reached the current target
		float ex = Goal.pose.position.x - msg.pose.pose.position.x ;
		float ey = Goal.pose.position.y - msg.pose.pose.position.y ;
		float rho = sqrt(ex*ex+ey*ey);
		if (abs(ex) < 0.1 and abs(ey) < 0.1) {
			//if the robot has reached the current target, publish the next target
			if (currentTarget < targets.size()) {
				Goal.pose.position.x = targets[currentTarget][0];
				Goal.pose.position.y = targets[currentTarget][1];
				goal_pub_.publish(Goal);
				currentTarget ++;
			}
		}
	}



};


int main(int argc, char** argv) {


	ros::init(argc, argv, "followTargets");
	ros::NodeHandle nh("~");
	FollowTargetsClass FT;

	ros::spin();
	return 0;
}

