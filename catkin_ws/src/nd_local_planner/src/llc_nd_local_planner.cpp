#include <math.h>

#include <pluginlib/class_list_macros.h>
#include <utility>
#include <string>
#include <chrono>

#include "llc_nd_local_planner.h"



//register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(p09_llc_nd_local_planner::LLCNDLocalPlanner, nav_core::BaseLocalPlanner)


using namespace std;

namespace p09_llc_nd_local_planner{
	struct Valley {
		int sector_rd; // sector corresponding to the rising discontinuity closest to goal
		int sector_od; // other discontinuity
		int sector_l; // left sector of the valley
		int sector_r; // right sector of the valley
		int distance_to_goal; // distance from goal to sector_rd in sectors
	};

	double euclideanDistance(const geometry_msgs::Pose pose1, const geometry_msgs::Pose pose2){
		double ex = pose2.position.x - pose1.position.x;
		double ey = pose2.position.y - pose1.position.y;

		return std::sqrt(ex*ex+ey*ey);
	}

	bool sort_valley(const Valley& l, const Valley& r) {
		return l.distance_to_goal < r.distance_to_goal;
	}

	/// ND Algotithm Functions
	

	double diffAngles(const double a1, const double a2){
		double diff = a1 - a2;
		if (diff > M_PI) return diff - 2*M_PI;
		if (diff < -M_PI) return 2*M_PI + diff;
		return diff;
	}

	int diffSectors(const int s1, const int s2, const int num_sectors) {
		int df = abs(s1 - s2);
		return min(df, num_sectors - df);
	}

	double clampAngle(const double angle) {
		if (angle > M_PI_2) return M_PI_2;
		else if (angle < -M_PI_2) return -M_PI_2;
		else return angle;
	}

	bool isValleyNavigable(const geometry_msgs::Pose& robot_pose, const double robot_radius,
                       const geometry_msgs::Pose& goal, const std::vector<geometry_msgs::Pose>& obstacles,
                       const std::vector<double>& obstacles_distance, const std::vector<double>& obstacles_angle) {
		// Calculate robot diameter
		const double robot_diameter = 2 * robot_radius;

		// Calculate goal angle
		double angle_goal = atan2(goal.position.y, goal.position.x);

		// Calculate goal distance
		double goal_distance = std::hypot(goal.position.x - robot_pose.position.x, goal.position.y - robot_pose.position.y);

		// Initialize vectors to store obstacle indices
		std::vector<int> right_obstacles, left_obstacles;

		// Iterate over obstacles
		for (size_t i = 0; i < obstacles.size(); i++) {
			// Check if obstacle is closer to the goal than the robot or if obstacle is further from the robot than the goal
			if (euclideanDistance(obstacles[i], goal) < robot_radius || obstacles_distance[i] > goal_distance) {
				continue;
			}

			// Calculate angle difference and check if obstacle is within the robot's field of view
			double angle_diff = diffAngles(obstacles_angle[i], angle_goal);
			if (abs(angle_diff) > M_PI_2 || abs((goal.position.x - robot_pose.position.x) * (obstacles[i].position.y - robot_pose.position.y) - 
				(goal.position.y - robot_pose.position.y) * (obstacles[i].position.x - robot_pose.position.x)) / goal_distance > robot_diameter) {
				continue;
			}

			// Add obstacle to the appropriate list
			(angle_diff >= 0 ? left_obstacles : right_obstacles).push_back(i);
		}

		// Check if any pair of obstacles from the left and right lists are too close to each other
		for (int i : left_obstacles) {
			for (int j : right_obstacles) {
				if (euclideanDistance(obstacles[i], obstacles[j]) <= robot_diameter) {
					return false;
				}
			}
		}

		// If no obstacles are too close, the valley is navigable
		return true;
	}

	double bisectorAngle(const int sector, const int num_sectors) {
		return M_PI - (2.0f * M_PI * (double)(sector+1)) / (double)num_sectors;
	}
	
	int angleToSector(const double angle, const int num_sectors) {
		return ceil((M_PI - angle)  * (num_sectors / 2.0f) / M_PI) - 1;
	}

	LLCNDLocalPlanner::LLCNDLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){
		initialize(name, tf, costmap_ros);
	}

	void LLCNDLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros){

		if (!initialized_) {

			costmap_ros_ = costmap_ros;
			costmap_ = costmap_ros_->getCostmap();
			tf_ = tf;

			ros::NodeHandle nh("~/" + name);
			ros::NodeHandle nh_local("~/local_costmap/");

			// nh_local.getParam("robot_radius", robot_radius_);
			nh.getParam("robot_radius_nd", robot_radius_);
			nh.getParam("distance_goal_factor", goal_factor_);
			nh.getParam("distance_robot_bounds", distance_bounds_);
			nh.getParam("max_linear_velocity", v_max_);
			nh.getParam("max_rotational_velocity", w_max_);
			nh.getParam("security_distance", security_distance_);
			nh.getParam("max_sensor_distance", distance_max_);
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
			ROS_WARN("LLCNDLocalPlanner: Global plan empty");
			return false;
		}

	   	return true;
	}

	bool LLCNDLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
	    //Compute the velocity command (v,w) for a differential-drive robot
		auto begin = std::chrono::steady_clock::now();

		if (!initialized_) {
			ROS_ERROR("THe planner has not been initialized.");
			return false;
		}

		// Get robot pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose,robot_global_pose;
		if (!costmap_ros_->getRobotPose(robot_global_pose)) {
			ROS_ERROR("Could not get robot pose");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			return false;
		}
		geometry_msgs::PoseStamped goal;

		// Get transform from global to robot
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
			ROS_ERROR("Error in lookupTransform");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
        	return false;
		}


		// Transform robot and goal pose to robot frame
		
		tf2::doTransform(robot_global_pose, robot_pose, globalToRobotTransform_);
		tf2::doTransform(global_plan_.back(), goal, globalToRobotTransform_);

		costmap_ = costmap_ros_->getCostmap();


		/// 1. Read obstacle information from the costmap
		std::vector<double> distances(num_sectors_, distance_max_ + 10.0f); // min distance to each sector
		std::vector<geometry_msgs::Pose> obstacles; // obstacle position in robot frame
		std::vector<double> dist_obst_robot; // distance obstacle - robot
		std::vector<double> angle_obst_robot; // angle obstacle - robot

		for (unsigned int x = 0; x < costmap_->getSizeInCellsX(); x++) {
			for (unsigned int y = 0; y < costmap_->getSizeInCellsY(); y++){
				if (costmap_->getCost(x, y) == costmap_2d::LETHAL_OBSTACLE) {
					// Distance to robot
					geometry_msgs::Pose obs_global_pose, obs_pose;					
					costmap_->mapToWorld(x, y, obs_global_pose.position.x, obs_global_pose.position.y);
					
					tf2::doTransform(obs_global_pose, obs_pose, globalToRobotTransform_);
					obstacles.push_back(obs_pose);
					

					double dist = euclideanDistance(obs_pose, robot_pose.pose);
					dist_obst_robot.push_back(dist);

					// Angle from robot
					double angle = atan2(obs_pose.position.y, obs_pose.position.x);
					angle_obst_robot.push_back(angle);
					int sector = angleToSector(angle, num_sectors_);

					// Update min distance sector
					if (dist < distances[sector]) distances[sector] = dist;
					
				}
			}
		}

		/// 2. Compute pnd and rnd
		const double robot_fit = 2 * robot_radius_;

		std::vector<double> pnd(num_sectors_, 0.0f);
		std::vector<double> rnd(num_sectors_, 0.0f);
		bool obstacle_in_front = false;

		for (int i = 0; i < num_sectors_; i++) {
			double dist = distances[i];
			if (dist > 0 && dist <= distance_max_) {
				pnd[i] = distance_max_ + robot_fit - dist;
				rnd[i] = distance_max_ + distance_bounds_ - dist;
				obstacle_in_front = true;
			}
		}

		double angle = atan2(goal.pose.position.y, goal.pose.position.x);
		int sector_goal = angleToSector(angle, num_sectors_);
		double goal_dist = euclideanDistance(goal.pose, robot_pose.pose);
		if (goal_dist < distances[sector_goal]) pnd[sector_goal] = 0;

		/// 3-4. Identify Gaps and Regions
		std::vector<Valley> valleys;

		if (!obstacle_in_front) {
			valleys.emplace_back(Valley{0, num_sectors_ - 1, 0, num_sectors_ - 1, 0});
		} else {
			/// 3. Identify Gaps
			std::vector<int> gaps; // guarda el indice del sector donde empieza el gap
			for (int i = 0; i < num_sectors_; i++) {
				int j = (i + 1) % num_sectors_;
				double d = pnd[i] - pnd[j]; 
				if (abs(d) > robot_fit) {
					gaps.push_back(d >= 0 ? j : i);
				}
			}

			/// 4. Identify Regions
			for (int i = 0; i < gaps.size(); i++) {
				int start_sector = gaps[i];
				int next_gap = (i + 1 ) % gaps.size();
				if (next_gap < gaps.size()) { // puede existir region
					int end_sector= gaps[next_gap];

					bool discontinuity = pnd[(start_sector - 1) % num_sectors_] > pnd[start_sector] ||
										pnd[(end_sector+ 1) % num_sectors_] > pnd[end_sector];

					if (!discontinuity) continue;

					Valley valley;
					valley.sector_l = start_sector;
					valley.sector_r = end_sector;
					int distance_start = diffSectors(start_sector, sector_goal, num_sectors_);
					int distance_end = diffSectors(end_sector, sector_goal, num_sectors_);
					if(distance_start < distance_end) {
						valley.sector_rd = start_sector;
						valley.sector_od = end_sector;
						valley.distance_to_goal = distance_start;
					} else {
						valley.sector_rd = end_sector;
						valley.sector_od = start_sector;
						valley.distance_to_goal = distance_end;
					}

					bool region = true;
					int idx = start_sector;

					while (idx != end_sector) {
						int next_idx = (idx + 1) % num_sectors_;
						if (abs(pnd[idx] - pnd[next_idx]) > robot_fit) {
							region = false;
							break;
						}
						idx = next_idx;
					}

					if (!region) continue;

					valleys.emplace_back(valley);
				}
			}
		}

		/// 5. Free walking area
		if (valleys.empty()){
			ROS_ERROR("No free space");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			return false;
		}

		// Sort the valleys by proximity to the goal
		std::sort(valleys.begin(), valleys.end(), &sort_valley);

		bool goal_in_valley = false;
		int idx_valley;

		// Find the first navigable valley
		idx_valley = std::find_if(valleys.begin(), valleys.end(), [&](const Valley& valley) {
			// Check if the goal is in the current valley
			if(valley.sector_l > valley.sector_r) {
				goal_in_valley = !(valley.sector_r < sector_goal && sector_goal < valley.sector_l);
			} else{
				goal_in_valley = valley.sector_l <= sector_goal && sector_goal <= valley.sector_r;
			}

			geometry_msgs::Pose goal_pose;
			// If the goal is in the valley, use the goal pose
			if (goal_in_valley) {
				goal_pose = goal.pose;
			} else {
				// If the goal is not in the valley, calculate a new goal pose based on the bisector of the valley
				int idx_1 = valley.sector_rd;
				int idx_2 = (valley.sector_rd == valley.sector_l ? idx_1 - 1 : idx_1 + 1) % num_sectors_;

				double dist = abs(distances[idx_1] - distances[idx_2]) / 2.0f + distances[idx_2];
				double angle = bisectorAngle(idx_1, num_sectors_);

				goal_pose.position.x = dist * cos(angle);
				goal_pose.position.y = dist * sin(angle);
			}

			// Check if the valley is navigable
			return isValleyNavigable(robot_pose.pose, robot_radius_, goal_pose, obstacles, dist_obst_robot, angle_obst_robot);
		}) - valleys.begin();

		Valley valley = valleys[idx_valley];

		// If no navigable valley is found, stop the robot and return false
		if (idx_valley == valleys.size()) {
			ROS_ERROR("No valley navigable");
			cmd_vel.linear.x = 0;
			cmd_vel.angular.z = 0;
			return false;
		}
		

		/// 6. Choose situation and compute action
		// rnd sectors that exceed the security nearness at both sides of sector_rd
		
		double distance_obstacle_left = distance_max_ + 10.0f, distance_obstacle_right = distance_max_ + 10.0f;
		double max_m_left = 0, max_m_right = 0;
		int sector_m_left = -1, sector_m_right = -1;
		for (int i = num_sectors_/4; i < 3*num_sectors_/4; i++) {
			if (rnd[i] > security_nearness_) {
				if (i > valley.sector_rd && rnd[i] > max_m_right) { // right
					sector_m_right = i;
					max_m_right = rnd[i];
					distance_obstacle_right = distances[i];
				} else if (i <= valley.sector_rd && rnd[i] > max_m_left) { // left
					sector_m_left = i;
					max_m_left = rnd[i];
					distance_obstacle_left = distances[i];
				}
			}
		}

		double v = 0.0, w = 0.0, th = 0.0;

		// Criterion 1: Safety criterion
		bool obs_left = (sector_m_left != -1);
		bool obs_right = (sector_m_right != -1);

		int sector_th;
		double distance_obstacle = std::min(distance_obstacle_left, distance_obstacle_right) - robot_radius_;

		if (obs_left || obs_right) { // Low Safety
			// Criterion 2: Dangerous obstacle distribution criterion
			if (obs_left && obs_right) { // Low Safety 2
				std::cout << "Low Safety 2" << std::endl;
				
				int sector_med = (sector_m_left + sector_m_right) / 2;
				int c = round( (1.0f - distance_obstacle / (distance_obstacle_right + distance_obstacle_left)));

				c = (distance_obstacle_left > distance_obstacle_right) ? -c : c;
				sector_th = sector_med + c;
			
			} else { // Low Safety 1
				std::cout << "Low Safety 1\n";

				int sign = (valley.sector_rd == valley.sector_l && obs_left) ? 1 : 
						(valley.sector_rd == valley.sector_r && obs_right) ? -1 : 0;
				if (sign == -1) sector_m_left = sector_m_right;

				int sector_p = (sign != 0) ? (wide_valley_ / 2) /2 * (1 - (float)(diffSectors(valley.sector_rd, sector_m_left, num_sectors_)) / (float)(num_sectors_/2)) + (wide_valley_ / 2) : 1;

				sector_th = valley.sector_rd + sign * sector_p;
			}
			
			// Translational Velocity
			v = v_max_ * (distance_obstacle / security_distance_) * abs(1 - abs(th) / M_PI_2);

		} else { // High Safety
			cout << "High safety" << endl;
			if (goal_in_valley) { // High Safety Goal in Region
				sector_th = sector_goal;
				std::cout << "Goal in Region" << std::endl;
			} else {
				// Criterion 4: Free walking area width criterion
				int valley_width = diffSectors(valley.sector_l, valley.sector_r, num_sectors_) + 1;
				if (valley_width >= wide_valley_) { // High Safety Wide Region
					sector_th = (valley.sector_rd != valley.sector_l) ? valley.sector_rd - wide_valley_ / 4 : valley.sector_rd + wide_valley_ / 4;
					std::cout << "Wide Region" << std::endl;
					
				} else { // High Safety Narrow Region
					std::cout << "Narrow Region\n";
					sector_th = (valley.sector_l > valley.sector_r) ? valley.sector_l + (valley_width - 1) / 2 : (valley.sector_l + valley.sector_r) / 2;                    
				}
			}
		}

		sector_th = sector_th % num_sectors_;
		th = bisectorAngle(sector_th, num_sectors_);
		th = clampAngle(th);

		// Translational Velocity
		v = v_max_ * abs(1 - abs(th) / M_PI_2);

		// Rotational Velocity
		w = w_max_ * th / M_PI_2;

		std::cout << "Theta: " << th << std::endl;
		std::cout << "V: " << v << " W: " << w << std::endl;
		cmd_vel.linear.x = v;
		cmd_vel.angular.z = w;

		auto end = std::chrono::steady_clock::now();
		unsigned int elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end-begin).count();
		std::cout << "Exec. time: " << elapsed << " ms\n";

		exec_time += elapsed;
		n_times++;
		std::cout << "Average Exec. time: " << (float)exec_time / (float)n_times << " ms\n";
		
		return true;
	}

	bool LLCNDLocalPlanner::isGoalReached(){
		//Check if the robot has reached the position and orientation of the goal

		if (! initialized_) {
			ROS_ERROR("This planner has not been initialized.");
			return false;
		}

		//Get robot and goal pose in the global frame of the local costmap
		geometry_msgs::PoseStamped robot_pose;
		costmap_ros_->getRobotPose(robot_pose);
		const geometry_msgs::PoseStamped goal = global_plan_.back();

		// bool goalReached = false;



		//implement here the condition(s) to have reached the goal
		float ex = robot_pose.pose.position.x - goal.pose.position.x;
		float ey = robot_pose.pose.position.y - goal.pose.position.y;
		float dist = sqrt(ex*ex + ey*ey);


		return dist < robot_radius_ * goal_factor_;
	}

};