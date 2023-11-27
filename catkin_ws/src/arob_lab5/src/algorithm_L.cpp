bool RRTPlanner::computeRRT(const std::vector<int> start, const std::vector<int> goal, 
                            std::vector<std::vector<int>>& sol){
    bool finished = false;
    int maxIterations = 30000;
    int count = 0;
    //Initialize random number generator
    srand(time(NULL));
        
    std::cout << "Start x: " << start[0] << "  y: " << start[1] << "\n";
    std::cout << "Goal x: " << goal[0] << "  y: " << goal[1] << "\n";

    
    // Initialize the tree with the starting point in map coordinates (root)
    TreeNode *itr_node = new TreeNode(start); 

    int max_cell_dist = int(max_dist_ / 0.032);

    std::cout << "Max distance in cells: " << max_cell_dist << "\n";

    std::vector<std::vector<int>> evaluated_points;

    //While goal is not reached
    
    while(!finished && count < maxIterations){
        
        int x_samp = int(rand() % cell_width_);
        int y_samp = int(rand() % cell_height_);
        
        std::vector<int> x_rand_point{(int)x_samp,(int)y_samp};
        
        TreeNode *random_node = new TreeNode(x_rand_point);
        TreeNode *x_near_node = random_node -> neast(itr_node);

        //x_near_node -> setParent(itr_node);
        std::vector<int> x_near_point = x_near_node->getNode();

        double ev_distance = distance(x_rand_point[0], x_rand_point[1], x_near_point[0], x_near_point[1]);

        std::vector<int> x_new_point = x_rand_point;

        if(ev_distance > max_cell_dist){
            double theta = atan2((x_rand_point[1] - x_near_point[1]),(x_rand_point[0] - x_near_point[0]));
            x_new_point[0] = x_near_point[0] + max_cell_dist*cos(theta);
            x_new_point[1] = x_near_point[1] + max_cell_dist*sin(theta);
        }

        if(std::find(evaluated_points.begin(), evaluated_points.end(), x_new_point) != evaluated_points.end()) {
            /* v contains x */
            count++;
            continue;
        }
        
        /* v does not contain x */
        evaluated_points.push_back(x_new_point);

        TreeNode *x_new_node = new TreeNode(x_new_point);

        if(obstacleFree(x_near_point[0], x_near_point[1], x_new_point[0], x_new_point[1])){
            //Debug
            // std::cout << "Sampled node x_rand: " << "\n";
            // random_node->printNode();
            // std::cout << "Nearest node in tree x_near: " << "\n";
            // x_near_node -> printNode();
            x_near_node -> appendChild(x_new_node);
            // std::cout << "Adding node xnew to the tree: " << "\n";
            // x_new_node->printNode();

            ev_distance = distance(x_new_point[0], x_new_point[1], goal[0], goal[1]);
            //std::cout << "Iteration: " << count << ", Distance to goal: " << ev_distance << std::endl; 
            if(obstacleFree(x_new_point[0], x_new_point[1], goal[0], goal[1]) && ev_distance <= max_cell_dist){
                sol = x_new_node->returnSolution();
                finished = true;
                std::cout << "finished plan!" << std::endl;
            }
            
        }

        count++;
    }


    ROS_WARN("Finished after %d iterations", count);

    itr_node->~TreeNode();

    return finished;
}