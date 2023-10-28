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
	std::vector<std::vector<float>> targets;
	std::vector<double> previousPose;
	int currentTarget; //index with the next target to reach


public:
	FollowTargetsClass(string targets_file) { //in the constructor you can read the targets from the text file
		//open the file
		inFile.open("/home/nerea/AR/catkin_ws/src/p09_arob_lab3/" + targets_file);
		if (!inFile) {
			cout << "Unable to open file";
			exit(1); // terminate with error
		}
		//read the targets from the file
		float x, y, z;

		string tmp;
		vector<string>* p = NULL;
		while (!inFile.eof()) {
			std::vector<float> target;
			getline(inFile, tmp, ';');
			x = stof(tmp);

			getline(inFile, tmp, ';');
			y = stof(tmp);
			
			getline(inFile, tmp, ';');
			z = stof(tmp);

			target.push_back(x);
			target.push_back(y);
			target.push_back(z);
			targets.push_back(target);
		}
		cout << "Targets: " << targets.size() << endl;

		
		//close the file
		inFile.close();
		//initialize the current target
		currentTarget = 0;
		//initialize the goal
		Goal.pose.position.x = targets[currentTarget][0];
		Goal.pose.position.y = targets[currentTarget][1];
		Goal.pose.position.z = targets[currentTarget][2];
		//initialize the publisher
		goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("command/pose", 1);
		//initialize the subscriber
		// review the topic name
		position_sub_ = nh_.subscribe("ground_truth/state", 1, &FollowTargetsClass::positionCb, this);
		
		//publish the goal
		goal_pub_.publish(Goal);
		currentTarget ++;
		previousPose = {0,0,0};

	}

	~FollowTargetsClass() {
	}

	//complete the class by adding the functio that you need

	void positionCb(const nav_msgs::Odometry& msg) {
		// cout << msg.pose.pose.position.x << " " << msg.pose.pose.position.y << " " << msg.pose.pose.position.z << endl;
		//check if the robot has reached the current target
		float ex = Goal.pose.position.x - msg.pose.pose.position.x ;
		float ey = Goal.pose.position.y - msg.pose.pose.position.y ;
		float ez = Goal.pose.position.z - msg.pose.pose.position.z ;
		if (abs(ex) < 0.02 and abs(ey) < 0.02 and abs(ez) < 0.02) {
			//if the robot has reached the current target, publish the next target
			if (currentTarget < targets.size()) {
				Goal.pose.position.x = targets[currentTarget][0];
				Goal.pose.position.y = targets[currentTarget][1];
				Goal.pose.position.z = targets[currentTarget][2];
				goal_pub_.publish(Goal);
				currentTarget ++;
			}
		}

		// if the robot is not moving and the robot has not reached the current target, publish the goal again
		if (!isMoving({msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z}) && (currentTarget <= targets.size() && ! (abs(ex) < 0.1 and abs(ey) < 0.1 and abs(ez) < 0.1))) {
			// cout << "Robot is not moving, publishing goal again" << endl;
			publishGoal();
		}

		previousPose = {msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z};
	}

	//Check if the robot is moving
	bool isMoving(std::vector<double> currentPose){
		//if the robot is moving, return true
		//if the robot is not moving, return false
		//you can use the distance between the current pose and the previous pose
		//if the distance is less than a threshold, the robot is not moving
		//if the distance is greater than a threshold, the robot is moving
		//you can use the function sqrt(pow(x,2)+pow(y,2)) to compute the distance between two points
		//you can use a threshold of 0.01
		return sqrt(pow(currentPose[0]-previousPose[0],2) + pow(currentPose[1]-previousPose[1],2) + pow(currentPose[2]-previousPose[2],2)) > 0.05 ? true : false;
	}

	// Publish the goal again
	void publishGoal(){
		goal_pub_.publish(Goal);
	}
};




int main(int argc, char** argv) {

	string filename = argv[1];
	ros::init(argc, argv, "followTargets3D");
	ros::NodeHandle nh("~");
	FollowTargetsClass FT(filename);

	ros::spin();
	return 0;
}

