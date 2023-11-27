#include <pluginlib/class_list_macros.h>
#include "AROB_lab5/rrt_global_planner.h"
#include <cmath>
#include <random>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RRTPlanner, nav_core::BaseGlobalPlanner)

//Default Constructor
namespace rrt_planner {

double distance(const unsigned int x0, const unsigned int y0, const unsigned int x1, const unsigned int y1){
    return std::sqrt((int)(x1-x0)*(int)(x1-x0) + (int)(y1-y0)*(int)(y1-y0));
}

RRTPlanner::RRTPlanner() : costmap_ros_(NULL), initialized_(false),
                            max_samples_(0.0){}

RRTPlanner::RRTPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    initialize(name, costmap_ros);
}

void RRTPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){

    if (!initialized_){
        ros::NodeHandle nh("~/" + name);
        ros::NodeHandle nh_local("~/local_costmap/");
        ros::NodeHandle nh_global("~/global_costmap/");

        // use markers to visualize the tree
        nh.param("visualize_markers", visualize_markers_, true);
        if (visualize_markers_){
            std::cout << "Initializing markers" << std::endl;
            marker_pub_ = nh.advertise<visualization_msgs::Marker>("/rrt_marker", 100);
        }

        nh.param("max_samples_", max_samples_, 30000.0);

        // nh.param("max_dist_", max_dist_, 0.5);

        //to make sure one of the nodes in the plan lies in the local costmap
        double width, height;
        nh_local.param("width", width, 3.0);
        nh_local.param("height", height, 3.0);
        max_dist_ = (std::min(width, height)/6.0);  //or any other distance within local costmap
        // max_dist_ = 0.1;

        nh_global.param("resolution", resolution_, 0.032);

        // std::cout << "Parameters: " << max_samples_ << ", " << dist_th_ << ", " << visualize_markers_ << ", " << max_dist_ << std::endl;
        // std::cout << "Local costmap size: " << width << ", " << height << std::endl;
        // std::cout << "Global costmap resolution: " << resolution_ << std::endl;

        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros->getCostmap();
        global_frame_id_ = costmap_ros_->getGlobalFrameID();

        initialized_ = true;

        std::cout << "Visualization markers: " << visualize_markers_ << std::endl;
        std::cout << costmap_->getSizeInCellsX() << ", " << costmap_->getSizeInCellsY() << std::endl;
        std::cout << "Max dist: " << max_dist_ << std::endl;
        std::cout << int(max_dist_/0.032) << std::endl;
    }
	else{
	    ROS_WARN("This planner has already been initialized... doing nothing.");
    }
}

bool RRTPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, 
                            std::vector<geometry_msgs::PoseStamped>& plan ){

    // std::cout << "RRTPlanner::makePlan" << std::endl;
    
    if (!initialized_){
        ROS_ERROR("The planner has not been initialized.");
        return false;
    }

	if (start.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The start pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), start.header.frame_id.c_str());
		return false;
	}

	if (goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
		ROS_ERROR("The goal pose must be in the %s frame, but it is in the %s frame.",
				  global_frame_id_.c_str(), goal.header.frame_id.c_str());
		return false;
	}
    
    plan.clear();
    costmap_ = costmap_ros_->getCostmap();  // Update information from costmap
    
    // Get start and goal poses in map coordinates
    unsigned int goal_mx, goal_my, start_mx, start_my;
    if (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, goal_mx, goal_my)){
        ROS_WARN("Goal position is out of map bounds.");
        return false;
    }    
    costmap_->worldToMap(start.pose.position.x, start.pose.position.y, start_mx, start_my);

    std::vector<int> point_start{(int)start_mx,(int)start_my};
    std::vector<int> point_goal{(int)goal_mx,(int)goal_my};    
  	std::vector<std::vector<int>> solRRT;
    bool computed = computeRRT(point_start, point_goal, solRRT);
    if (computed){        
        getPlan(solRRT, plan);
        // add goal
        plan.push_back(goal);
    }else{
        ROS_WARN("No plan computed");
    }

    return computed;
}

bool RRTPlanner::computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol){
    bool finished = false;

    //Initialize random number generator
    srand(time(NULL));
        
    // Initialize the tree with the starting point in map coordinates
    TreeNode *root = new TreeNode(start); 
    // TreeNode *itr_node = root;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis_x(0, costmap_->getSizeInCellsX());
    std::uniform_real_distribution<> dis_y(0, costmap_->getSizeInCellsY());

    int max_dist_cells_ = int(max_dist_/0.032);

    std::cout << "Max dist in cells: " << max_dist_cells_ << std::endl;
    
    int cellWidth = int(16/0.032);
    int cellHeight = int(16/0.032);

    // implement RRT algorithm here
    int count = 0;
    std::cout << "Max samples: " << int(max_samples_) << std::endl;
    while (!finished && count < int(max_samples_) ){
        int c2 = 0;
        int x_rand = int(rand() % costmap_->getSizeInCellsX());
        int y_rand = int(rand() % costmap_->getSizeInCellsY());
        std::vector<int> point_rand{x_rand, y_rand};

        TreeNode *rand_node = new TreeNode(point_rand);

        // Find the closest node in the tree to the random point
        TreeNode *nearest_node = rand_node->neast(root);  
        std::vector<int> nearest_point = nearest_node->getNode();

        std::vector<int> new_point;
        // if the distance between the nearest node and the random point is greater than max_dist_
        // then the new point is max_dist_ away from the nearest node in the direction of the random point
        if (distance(nearest_point[0], nearest_point[1], point_rand[0], point_rand[1]) > max_dist_cells_){
            double theta = atan2(point_rand[1] - nearest_point[1], point_rand[0] - nearest_point[0]);
            new_point[0] = nearest_point[0] + max_dist_cells_*cos(theta);
            new_point[1] = nearest_point[1] + max_dist_cells_*sin(theta);
        }else{
            new_point = point_rand;
        }

        // check if the line between the nearest node and the random point is free of collision
        if (obstacleFree(nearest_point[0], nearest_point[1], new_point[0], new_point[1])){
            
            c2 ++;

            // add the new point to the tree
            TreeNode *new_node = new TreeNode(new_point);
            nearest_node->appendChild(new_node);
            // std::cout << "size of tree: " << root->size() << std::endl;
            // new_node->setParent(nearest_node);
            // itr_node = new_node;

            // check if the new node is close enough to the goal
            if (distance(new_point[0], new_point[1], goal[0], goal[1]) <= max_dist_cells_ && obstacleFree(new_point[0], new_point[1], goal[0], goal[1])){
                TreeNode *goal_node = new TreeNode(goal);
                new_node->appendChild(goal_node);
                // goal_node->setParent(new_node);
                // root->printTree();
                sol = new_node->returnSolution();
                std::cout << "Solution size" << sol.size() << std::endl;
                std::cout << "size of tree: " << root->size() << std::endl;
                std::cout << "Iterations: " << count << std::endl;
                std::cout << "shoud be " << c2 << " nodes" << std::endl;
                finished = true;
                break;
            }
        }
        count ++;
    }
    ROS_INFO("Iterations: %d", count);



    // publish the tree into rviz with markers
    if (visualize_markers_ && finished){
        std::cout << "Publishing markers" << std::endl;

        visualization_msgs::Marker points, line_strip, line_list;
        points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = global_frame_id_;
        points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
        points.ns = line_strip.ns = line_list.ns = "rrt";
        points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
        points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;

        points.id = 0;
        line_strip.id = 1;
        line_list.id = 2;

        points.type = visualization_msgs::Marker::POINTS;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        line_list.type = visualization_msgs::Marker::LINE_LIST;

        // POINTS markers use x and y scale for width/height respectively
        points.scale.x = 0.2;
        points.scale.y = 0.2;

        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        line_list.scale.x = 0.1;

        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;

        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;

        // Line list is red
        line_list.color.r = 1.0;
        line_list.color.a = 1.0;



        // Create the vertices for the points and lines
        for(std::vector<int> node : sol){
            geometry_msgs::Point p;
            costmap_->mapToWorld((unsigned int)node[0], (unsigned int)node[1], p.x, p.y);
            p.z = 0.0;
            points.points.push_back(p);
            line_strip.points.push_back(p);

            // The line list needs two points for each line
            line_list.points.push_back(p);
            p.z += 0.1;
            line_list.points.push_back(p);
        }

        // Publish the nodes marker
        marker_pub_.publish(points);    
        marker_pub_.publish(line_strip);
        marker_pub_.publish(line_list);
    }

    delete root;

    return finished;
}

bool RRTPlanner::obstacleFree(const unsigned int x0, const unsigned int y0, 
                            const unsigned int x1, const unsigned int y1){
    //Bresenham algorithm to check if the line between points (x0,y0) - (x1,y1) is free of collision

    int dx = x1 - x0;
    int dy = y1 - y0;

    int incr_x = (dx > 0) ? 1.0 : -1.0;
    int incr_y = (dy > 0) ? 1.0 : -1.0;

    unsigned int da, db, incr_x_2, incr_y_2;
    if (abs(dx) >= abs(dy)){
        da = abs(dx); db = abs(dy);
        incr_x_2 = incr_x; incr_y_2 = 0;
    }else{
        da = abs(dy); db = abs(dx);
        incr_x_2 = 0; incr_y_2 = incr_y;
    }

    int p = 2*db - da;
    unsigned int a = x0; 
    unsigned int b = y0;
    unsigned int end = da;
    for (unsigned int i=0; i<end; i++){
        if (costmap_->getCost(a, b) != costmap_2d::FREE_SPACE){  // to include cells with inflated cost
            return false;
        }else{
            if (p >= 0){
                a += incr_x;
                b += incr_y;
                p -= 2*da;
            }else{
                a += incr_x_2;
                b += incr_y_2;
            }
            p += 2*db;
        }
    }

    return true;
}

void RRTPlanner::getPlan(const std::vector<std::vector<int>> sol, std::vector<geometry_msgs::PoseStamped>& plan){

    for (auto it = sol.rbegin(); it != sol.rend(); it++){
        std::vector<int> point = (*it);
        geometry_msgs::PoseStamped pose;

        costmap_->mapToWorld((unsigned int)point[0], (unsigned int)point[1], 
                            pose.pose.position.x, pose.pose.position.y);
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = global_frame_id_;
        pose.pose.orientation.w = 1;
        plan.push_back(pose);

    }
}

};
