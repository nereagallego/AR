#ifndef LLC_LOCAL_PLANNER_H
#define LLC_LOCAL_PLANNER_H

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <angles/angles.h>
#include <chrono>
#include <vector>

namespace p09_llc_nd_local_planner{

class LLCNDLocalPlanner: public nav_core::BaseLocalPlanner{

	costmap_2d::Costmap2DROS* costmap_ros_;
	costmap_2d::Costmap2D* costmap_;
	tf2_ros::Buffer* tf_;	//for tf transformations

	std::vector<geometry_msgs::PoseStamped> global_plan_;

	double robot_radius_;

	bool initialized_;

	// ND parameters
	double v_max_;
	double w_max_;
	double goal_factor_;
	double distance_bounds_;

	// Sensor parameters
	double distance_max_;

	// Control parameters
	double security_distance_;
	double security_nearness_;

	int ls2_diff_;

	int num_sectors_;
	int wide_valley_;


	long unsigned int exec_time = 0;
	long unsigned int n_times = 0;

public:

	LLCNDLocalPlanner() : costmap_(NULL), initialized_(false){};
	LLCNDLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);

	/*override functions from interface nav_core::BaseLocalPlanner*/
	void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros);
	bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);
	bool isGoalReached();

private:

};

};

#endif
