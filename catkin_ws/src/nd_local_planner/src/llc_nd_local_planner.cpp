#include <pluginlib/class_list_macros.h>

#include "llc_nd_local_planner.h"



//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(p09_llc_nd_local_planner::LLCNDLocalPlanner,
nav_core::BaseLocalPlanner)


using namespace std;

namespace p09_llc_nd_local_planner{

	struct Valley {
		int sector_rd; // sector of the rising discontinuity closest to the goal
		int sector_od; // other discontunity
		int sector_l; // left sector
		int sector_r; // right sector
		int distance_to_goal; // distance to the goal from sector_rd in sectors
	}

	double euclideanDistance(const geometry_msgs::Pose pose1, const geometry_msgs::Pose pose2){
		double ex = pose2.position.x - pose1.position.x;
		double ey = pose2.position.y - pose1.position.y;

		return std::sqrt(ex*ex+ey*ey);
	}

	bool sort_valleys(const Vallet& v1, const Vallet& v2){
		return v1.distance_to_goal < v2.distance_to_goal;
	}

	double diffAngles(const double a1, const double a2){
		double diff = a1 - a2;
		if (diff > M_PI) diff = diff - 2*M_PI;
		if (diff < -M_PI) diff = 2*M_PI + diff;
		return diff;
	}

	int diffSectors(const int s1, const int s2, const int n_sectors){
		int diff = abs(s1 - s2);
		return min(diff, n_sectors - diff);
	}

	double clampAngle(const double a){
		if (a > M_PI_2) a = M_PI_2;
		if (a < -M_PI_2) a = - M_PI_2;
		return a;
	}

	double bisectorAngle(const int sector, const int n_sectors){
		return M_PI - (2.0f * M_PI * (double)(sector+1) ) / (double)n_sectors;
	}

	int angleSector(const double angle, const int n_sectors){
		return (int)ceil((M_PI - angle) * (n_sectors / 2.0f)/ M_PI) - 1;
	}

	bool isValleyNavigable(const  geometry_msgs::Pose &robot_pose, const double robot_radius, const geometry_msgs::Pose &goal, const std::vectpr<geometry_msgs::Pose>& obstacles, const std::vector<double>& obstacles_distance, const std::vector<double>& obstacle_angle){
		const double robot_fit = robot_radius * 2.0f;

		double angle_goal = atan2(goal.position.y, goal.position.x);

		double ex = goal.position.x - robot_pose.position.x;
		double ey = goal.position.y - robot_pose.position.y;
		double goal_distance = std::sqrt(ex*ex + ey*ey);

		std::vector<int> right_obstacles, left_obstacles;
		for(int i = 0; i < obstacles.size(); i++){
			// check if goal cannot be reached. Goal too close to obstacle
			double distance_obstacle_goal = euclideanDistance(ostacles[i], goal);
			if (distance_obstacle_goal < robot_radius) {
				ROS_WARN("Goal too close to obstacle");
				return false;
			}	

			// check if obstacle is safe

			// check if goal is closer than obstacle
			double distance_obstacle_robot = obstacles_distance[i];
			if(distance_obstacle_robot > distance_obstacle_goal) continue;

			// ignore obstacles that are not in the direction of the goal
			double angle_diff = diffAngles(obstacle_angle[i], angle_goal);
			if(abs(angle_diff) > M_PI_2) continue;

			// compute direct path to goal. If the obstacle is not in the way, ignore it
			double obstaclesx = obstacles[i].position.x - robot_pose.position.x;
			double obstaclesy = obstacles[i].position.y - robot_pose.position.y;
			double d_oP = abs(ex*obstaclesy - ey*obstaclesx) / goal_distance;
			if (d_oP > robot_fit) continue;

			// add the obstacle to the right or left
			if (angle_diff > 0) right_obstacles.push_back(i);
			else left_obstacles.push_back(i);

		}

		return true;
	}

	LLCNDLocalPlanner::LLCNDLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, tf, costmap_ros);
	}

	void LLCNDLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

		if(!initialized_){

			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();
			tf_ = tf;

			ros::NodeHandle nh("~/" + name);
			ros::NodeHandle nh_local("~/local_costmap/");

			// nh_local.getParam("robot_radius", robot_radius_);
			nh.getParam("robot_radius", robot_radius_);
			nh.getParam("distance_robot_bounds", distance_robot_bounds_);
			nh.getParam("distance_goal_factor", goal_factor_);
			nh.getParam("max_linear_velocity", v_max_);
			nh.getParam("ls2_diff", ls2_diff_);
			nh.getParam("max_sensor_distance", distance_max_);
			nh.getParam("max_rotational_velocity", w_max_);
			nh.getParam("security_distance", security_distance_);
			security_nearness_ = distance_max_ - security_distance_;
			nh.getParam("num_sectors", num_sectors_);
			int wide_angle;
			nh.getParam("wide_area_angle", wide_angle);
			wide_valley_ = num_sectors_ / (360 / wide_angle);

			initialized_ = true;

		}else{
			ROS_WARN("This planner has already been initialized, doing nothing.");
		}

	}

	bool LLCNDLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan){

		// std:cout << "setPlan ..." << std::endl;
		
		if(!initialized_){
			ROS_ERROR("THe planner has not been initialized.");
			return false;
		}

		//reset the global plan
		global_plan_.clear();

		// update obstacle info from costmap
		costmap_ = costmap_ros_->getCostmap();

		//Prune the plan to store the set of points within the local costmap
		for (auto it = plan.begin(); it != plan.end(); ++it){

			unsigned mx, my;
			if (costmap_->worldToMap((*it).pose.position.x, (*it).pose.position.y, mx, my))
				global_plan_.push_back((*it));
		}

		if (global_plan_.empty()){
			ROS_WARN("Global plan empty");
			return false;
		}

	   	return true;
	}


	// TODO: implement this method
	bool LLCNDLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
	    //Compute the velocity command (v,w) for a differential-drive robot

		// std::cout << "ComputeVelocityCommands ..." << std::endl;

		if(!initialized_){
			ROS_ERROR("THe planner has not been initialized.");
			return false;
		}

		//Get robot and goal pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		geometry_msgs::PoseStamped goal = global_plan_.back();

		costmap_ = costmap_ros_->getCostmap();

		//Get transform from global to robot frame
		geometry_msgs::TransformStamped globalToRobotTransform_, robotToGlobalTransform_;
		try {
			globalToRobotTransform_ = tf_->lookupTransform(costmap_ros_->getBaseFrameID(),
								costmap_ros_->getGlobalFrameID(),
								ros::Time(0),
								ros::Duration(0.5)
								);
			robotToGlobalTransform_ = tf_->lookupTransform(costmap_ros_->getGlobalFrameID(),
								costmap_ros_->getBaseFrameID(),
								ros::Time(0),
								ros::Duration(0.5)
								);
		} catch(tf2::TransformException& e) {
			ROS_ERROR("NDLocalPlanner: Error in lookupTransform");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
        	return false;
		}

		// 1) Read obstacles from costmap
		std::vector<double> distances(num_sectors_, distance_max_ + 10.0f);
		std::vector<geometry_msgs::Pose> obstacles; // obstacle positions
		std::vector<double> dist_obst_robot; // distance from obstacle to robot
		std::vector<double> angle_obst_robot; // angle from obstacle to robot

		for (int i = 0; i < costmap_->getSizeInCellsX(); i++){
			for (int j = 0; j < costmap_->getSizeInCellsY(); j++){
				if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE){
					// Distance to robot
					geometry_msgs::Pose obst_global_pose, obst_pose;
					costmap_->mapToWorld(i, j, obst_global_pose.position.x, obst_global_pose.position.y);

					// Transform obstacle pose to robot frame
					tf2::doTransform(obst_global_pose, obst_pose, globalToRobotTransform_);
					obstacles.push_back(obst_pose);

					double dist = euclideanDistance(robot_pose.pose, obst_pose);
					dist_obst_robot.push_back(dist);

					double angle = atan2(obst_pose.position.y, obst_pose.position.x);
					angle_obst_robot.push_back(angle);

					int sector = angleSector(angle, num_sectors_);
					if (dist < distances[sector]) distances[sector] = dist;
				}
			}
		}

		// 2) Compute PND and RND
		const double robot_fit = robot_radius_ * 2;

		std::vector<double> pnd(num_sectors_, 0.0f);
		std::vector<double> rnd(num_sectors_, 0.0f);
		bool obstacle_in_front = false;

		for (int i = 0; i < num_sectors_; i++){
			double dist = distances[i];
			if (dist <= distance_max_ && dist > 0){
				pnd[i] = distance_max_ + robot_fit - dist;
				rnd[i] = distance_max_ + distance_robot_bounds_ - dist;
				obstacle_in_front = true;
			}
		}

		double angle = atan2(goal.pose.position.y, goal.pose.position.x);
		int sector_goal = angleSector(angle, num_sectors_);
		double goal_distance = euclideanDistance(robot_pose.pose, goal.pose);
		if (goal_distance < distances[sector_goal]) pnd[sector_goal] = 0.0f;

		// 3) Identity gaps and regions
		std::vector<Valley> valleys;

		if(obstacle_in_front){
			// Indentity gaps
			std::vector<int> gaps;
			for(int i = 0; i < num_sectors_; i++){
				int j = (i + 1) % num_sectors_;
				double d = pnd[i] - pnd[j];
				if (abs(d) > robot_fit){
					if(d >= 0) gaps.push_back(i);
					else gaps.push_back(j);
				}
			}

			// Identify regions
			for(int i = 0; i < gaps.size(); i ++){
				int start_sector = gaps[i];
				int next_gap = (i + 1) % gaps.size();
				if (next_gap < gaps.size()){
					int end_sector = gaps[next_gap];
					bool discontinuity = pnd[(start_sector -1) % num_sectors_] > pnd[start_sector] || pnd[(end_sector + 1) % num_sectors_] > pnd[end_sector];

					if(discontinuity) continue;

					Valley valley;
					valley.sector_l = start_sector;
					valley.sector_r = end_sector;

					int distance_start = diffSectors(start_sector, sector_goal, num_sectors_);
					int distance_end = diffSectors(end_sector, sector_goal, num_sectors_);
					if(distance_start < distance_end){
						valley.sector_rd = start_sector;
						valley.sector_od = end_sector;
						valley.distance_to_goal = distance_start;
					}else{
						valley.sector_rd = end_sector;
						valley.sector_od = start_sector;
						valley.distance_to_goal = distance_end;
					}

					bool region = true;
					int idx = start_sector;

					while(idx != end_sector){
						int next_idx = (idx + 1) % num_sectors_;
						if(abs(pnd[idx] - pnd[next_idx]) > robot_fit){
							region = false;
							break;
						}
						idx = next_idx;
					}

					if(!region) continue;

					valleys.push_back(valley);
				}
			}
			
		} else {
			// No obstacle in front
			Valley valley;
			valley.sector_rd = 0;
			valley.sector_od = num_sectors_ - 1;
			valley.sector_l = 0;
			valley.sector_r = num_sectors_ - 1;
			valley.distance_to_goal = 0;
			valleys.push_back(valley);
		}

		// 5) Free space
		if (valleys.size() == 0){
			// No free space
			ROS_ERROR("No free space");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			return true;
		}

		std::sort(valleys.begin(), valleys.end(), &sort_valleys); // sort valleys by distance to goal

		Valley valley;
		bool goal_in_valley = false;
		int idx_valley;

		for (idx_valley = 0; idx_valley < valleys.size(); idx_valley++){
			valley = valleys[idx_valley];
			
			if(valley.sector_l > valley.sector_r){
				goal_in_valley = !(valley.sector_r < sector_goal && sector_goal < valley.sector_l);
			} else {
				goal_in_valley = valley.sector_l < sector_goal && sector_goal < valley.sector_r;
			}

			geometry_msgs::Pose goal_pose;
			if(goal_in_valley){
				goal_pose = goal.pose;
			} else {
				int idx_1 = valley.sector_rd;
				int idx_2 = ((valley.sector_rd == valley.sector_l) ? idx_1 -1 : idx_1 + 1) % num_sectors_;

				double dist = abs(distances[idx_1] - distances[idx_2]) / 2.0f + distances[idx_2];
				double angle = bisectorAngle(idx_1, num_sectors_);
				
				goal_pose.position.x = dist * cos(angle);
				goal_pose.position.y = dist * sin(angle);
			
			}


		}
		

		return true;
	}

	bool LLCNDLocalPlanner::isGoalReached(){
		//Check if the robot has reached the position and orientation of the goal


		// std::cout << "isGoalReached ..." << std::endl;

		if (! initialized_) {
			ROS_ERROR("This planner has not been initialized.");
			return false;
		}

		//Get robot and goal pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		const geometry_msgs::PoseStamped goal = global_plan_.back();

		float dist = euclideanDistance(robot_pose.pose, goal.pose);

		bool goalReached = dist < rho_th_;




		return goalReached;		
	}

};