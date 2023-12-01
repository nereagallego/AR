#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/MarkerArray.h>

#include <iostream>
#include <sstream>
#include <stdio.h> 
#include <math.h>
#include <fstream>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/QR>

#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

using namespace std;

class drone_race {

    std::vector<geometry_msgs::Pose> gates;

    //Trajectory attributes
    mav_trajectory_generation::Trajectory trajectory;

    //ROS publishers-suscribers
    ros::NodeHandle nh_;
    ros::Publisher pub_traj_markers_;
    ros::Publisher pub_traj_vectors_;
    ros::Publisher pub_gate_markers_;

    //Id markers
    int id_marker = 0;

    public:

    drone_race() {

        // create publisher for RVIZ markers
        pub_traj_markers_ =
            nh_.advertise<visualization_msgs::MarkerArray>("trajectory_markers", 0);
        pub_traj_vectors_ =
            nh_.advertise<visualization_msgs::MarkerArray>("trajectory_vectors", 0);
        pub_gate_markers_ =
            nh_.advertise<visualization_msgs::MarkerArray>("gate_markers", 0);
	}

    ~drone_race() {
    }

    int readGates(string file) {
        //Open the file
        ifstream inputFile;
	    inputFile.open(file, ifstream::in);
	    if (!inputFile) {
        	cerr << "Error opening the file." << endl;
        	return -1;
	    }

        gates.clear();
        geometry_msgs::Pose tempPose;
        double yaw = 0;
        std::string line;
        while (std::getline(inputFile, line))
        {
            std::istringstream iss(line);
            iss >> tempPose.position.x;
            iss >> tempPose.position.y;
            iss >> tempPose.position.z;
            iss >> yaw;
            tempPose.orientation = RPY_to_quat(0, 0, yaw);
            gates.push_back(tempPose);
        }

        // Close the file
        inputFile.close();
        return 1;
    }

    int drawGates() {
        int id = 0;
        for (geometry_msgs::Pose gate : gates) {
            draw_gate_markers(gate);
        }
        return 1;
    }

    void generate_trajectory_example() {
        //constants
        const int dimension = 3; //we only compute the trajectory in x, y and z
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP

        // Definition of the trajectory beginning, end and intermediate constraints
        mav_trajectory_generation::Vertex::Vector vertices;
        mav_trajectory_generation::Vertex start(dimension), middle(dimension), end(dimension);
        start.makeStartOrEnd(Eigen::Vector3d(0,0,1), derivative_to_optimize);
        vertices.push_back(start);

        //Position constraint
        middle.addConstraint(mav_trajectory_generation::derivative_order::POSITION, Eigen::Vector3d(1,2,3));
        //Velocity constraint (optional)
        middle.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, Eigen::Vector3d(1,0,0));
        vertices.push_back(middle);

        end.makeStartOrEnd(Eigen::Vector3d(2,1,5), derivative_to_optimize);
        vertices.push_back(end);
        
        // Provide the time constraints on the vertices
        //Automatic time computation
        std::vector<double> segment_times;
        const double v_max = 2.0;
        const double a_max = 2.0;
        segment_times = estimateSegmentTimes(vertices, v_max, a_max);
        cout << "Segment times = " << segment_times.size() << endl;
        /*for (int i=0; i< segment_times.size() ; i++) {
            cout << "Time " << i << " = " << segment_times[i] << endl;
        }*/
        //Manual time computation
        segment_times.clear();
        segment_times.push_back(3.5); // This is the time required to go from vertex 0 to vertex 1
        segment_times.push_back(2.5); // This is the time required to go from vertex 1 to vertex 2
        
        // Solve the optimization problem
        const int N = 10; //Degree of the polynomial, even number at least two times the highest derivative
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        //Obtain the trajectory
        opt.getTrajectory(&trajectory);
        //Sample the trajectory (to obtain positions, velocities, etc.)
        mav_msgs::EigenTrajectoryPoint::Vector states;
        double sampling_interval = 0.01; //How much time between intermediate points
        bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
        // Example to access the data
        cout << "Trajectory time = " << trajectory.getMaxTime() << endl;
        cout << "Number of states = " << states.size() << endl;
        cout << "Position (world frame) " << 3 << " X = " << states[2].position_W[0] << endl;
        cout << "Velocity (world frame) " << 3 << " X = " << states[2].velocity_W[0] << endl;

        // Default Visualization
        visualization_msgs::MarkerArray markers;
        double distance = 0.25; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        pub_traj_vectors_.publish(markers);

        //AROB visualization
        draw_trajectory_markers();
    }

    void generate_trajectory() {
        //constants
        const int dimension = 3; //we only compute the trajectory in x, y and z
        const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP; //POSITION, VELOCITY, ACCELERATION, JERK, SNAP

        // Definition of the trajectory beginning, end and intermediate constraints
        mav_trajectory_generation::Vertex::Vector vertices;
        // INCLUDE YOUR CODE HERE

        // Provide the time constraints on the vertices
        std::vector<double> segment_times;
        // INCLUDE YOUR CODE HERE
        
        // Solve the optimization problem
        const int N = 10; //Degree of the polynomial, even number at least two times the highest derivative
        mav_trajectory_generation::PolynomialOptimization<N> opt(dimension);
        opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
        opt.solveLinear();

        //Obtain the trajectory
        trajectory.clear();
        opt.getTrajectory(&trajectory);
        //Sample the trajectory (to obtain positions, velocities, etc.)
        mav_msgs::EigenTrajectoryPoint::Vector states;
        double sampling_interval = 0.1; //How much time between intermediate points
        bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);
        // Example to access the data
        cout << "Trajectory time = " << trajectory.getMaxTime() << endl;
        cout << "Number of states = " << states.size() << endl;
        cout << "Position (world frame) " << 3 << " X = " << states[2].position_W[0] << endl;
        cout << "Velocity (world frame) " << 3 << " X = " << states[2].velocity_W[0] << endl;
        
        // Default Visualization
        visualization_msgs::MarkerArray markers;
        double distance = 0.25; // Distance by which to seperate additional markers. Set 0.0 to disable.
        std::string frame_id = "world";
        mav_trajectory_generation::drawMavTrajectory(trajectory, distance, frame_id, &markers);
        pub_traj_vectors_.publish(markers);

        //AROB visualization
        draw_trajectory_markers();

        // Generate list of commands to publish to the drone
        // INCLUDE YOUR CODE HERE
    }

    void send_command() {
        // INCLUDE YOUR CODE TO PUBLISH THE COMMANDS TO THE DRONE
    }

    private: 
        Eigen::Matrix<double, 3, 3> RPY_to_R_matrix(double roll, double pitch, double yaw) {
            Eigen::AngleAxis<double> rollAngle(roll, Eigen::Matrix<double, 1, 3>::UnitX());
            Eigen::AngleAxis<double> pitchAngle(pitch, Eigen::Matrix<double, 1, 3>::UnitY());
            Eigen::AngleAxis<double> yawAngle(yaw, Eigen::Matrix<double, 1, 3>::UnitZ());

            Eigen::Matrix<double, 3, 3> R;

            Eigen::Quaternion<double> q = yawAngle * rollAngle * pitchAngle;

            R = q.matrix();

            return (R);
        }

        Eigen::Matrix<double, 3, 3> quat_to_R_matrix(geometry_msgs::Quaternion q) {
            double roll, pitch, yaw;
            tf2::Quaternion quat_tf;
            tf2::fromMsg(q, quat_tf);
            Eigen::Matrix<double, 3, 3> mat_res;
            tf2::Matrix3x3(quat_tf).getRPY(roll, pitch, yaw);

            return RPY_to_R_matrix(roll, pitch, yaw);
        }

        geometry_msgs::Quaternion RPY_to_quat(double roll, double pitch, double yaw) {
            tf2::Quaternion quaternion_tf2;
            quaternion_tf2.setRPY(roll, pitch, yaw);
            geometry_msgs::Quaternion quaternion = tf2::toMsg(quaternion_tf2);
            return quaternion;
        }

        void draw_goal_marker(mav_trajectory_generation::Vertex goal){
            Eigen::VectorXd pos;
            goal.getConstraint(mav_trajectory_generation::derivative_order::POSITION,&pos);
            visualization_msgs::Marker marker_aux;
            marker_aux.header.frame_id = "world";
            marker_aux.header.stamp = ros::Time(0);
            marker_aux.id = id_marker;
            id_marker++;
            marker_aux.ns = "point";
            marker_aux.type = visualization_msgs::Marker::CUBE;
            marker_aux.pose.position.x = pos(0);
            marker_aux.pose.position.y = pos(1);
            marker_aux.pose.position.z = pos(2);
            marker_aux.pose.orientation.x = 0;
            marker_aux.pose.orientation.y = 0;
            marker_aux.pose.orientation.z = 0;
            marker_aux.pose.orientation.w = 1;
            marker_aux.scale.x = 0.1;
            marker_aux.scale.y = 0.1;
            marker_aux.scale.z = 0.1;
            marker_aux.color.r = 1.0f;
            marker_aux.color.g = 0.0f;
            marker_aux.color.b = 0.0f;
            marker_aux.color.a = 1.0;
            marker_aux.lifetime = ros::Duration();
            visualization_msgs::MarkerArray marker_array;
            marker_array.markers.push_back(marker_aux);
            pub_traj_markers_.publish(marker_array);
        }

        void draw_trajectory_markers(){
            visualization_msgs::MarkerArray markers;
            int derivative_order = mav_trajectory_generation::derivative_order::POSITION;
            double sampling_time = 0.1;
            mav_msgs::EigenTrajectoryPoint::Vector states;
            double sampling_interval = 0.1;
            bool success = mav_trajectory_generation::sampleWholeTrajectory(trajectory, sampling_interval, &states);   
            for(int i=0; i< states.size(); i++) {
                visualization_msgs::Marker marker_aux;
                marker_aux.header.frame_id = "world";
                //marker_aux.header.stamp = ros::Time::now();
                marker_aux.header.stamp = ros::Time(0);
                marker_aux.id = id_marker;
                id_marker++;
                marker_aux.ns = "point";
                marker_aux.type = visualization_msgs::Marker::CUBE;
                marker_aux.pose.position.x = states[i].position_W[0] ;
                marker_aux.pose.position.y = states[i].position_W[1] ;
                marker_aux.pose.position.z = states[i].position_W[2] ;
                marker_aux.pose.orientation.x = 0;
                marker_aux.pose.orientation.y = 0;
                marker_aux.pose.orientation.z = 0;
                marker_aux.pose.orientation.w = 1;
                marker_aux.scale.x = 0.03;
                marker_aux.scale.y = 0.03;
                marker_aux.scale.z = 0.03;
                marker_aux.color.r = 0.0f;
                marker_aux.color.g = 0.0f;
                marker_aux.color.b = 1.0f;
                marker_aux.color.a = 1.0;
                marker_aux.lifetime = ros::Duration();
                markers.markers.push_back(marker_aux);
            }
            pub_traj_markers_.publish(markers);
        }

        void draw_gate_markers(geometry_msgs::Pose gate){
            visualization_msgs::Marker marker;
            visualization_msgs::Marker marker_yellowSphere;
            visualization_msgs::MarkerArray marker_array;
            visualization_msgs::Marker line_marker;
            //std::vector<visualization_msgs::Marker> line_marker_vector;
            
            Eigen::Matrix<double, 3, 3> rotate_gate = quat_to_R_matrix(gate.orientation);
            Eigen::Matrix<double, 3, 1> pos_gate(gate.position.x, gate.position.y, gate.position.z);

            marker.header.frame_id = "world";  // Change this frame_id according to your setup
            marker.header.stamp = ros::Time::now();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.ns = "corner";
            marker.scale.x = 0.2;
            marker.scale.y = 0.2;
            marker.scale.z = 0.2;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.lifetime = ros::Duration();

            marker_yellowSphere.header.frame_id = "world";  // Change this frame_id according to your setup
            marker_yellowSphere.header.stamp = ros::Time::now();
            marker_yellowSphere.type = visualization_msgs::Marker::SPHERE;
            marker_yellowSphere.action = visualization_msgs::Marker::ADD;
            marker_yellowSphere.ns = "corner";
            marker_yellowSphere.scale.x = 0.2;
            marker_yellowSphere.scale.y = 0.2;
            marker_yellowSphere.scale.z = 0.2;
            marker_yellowSphere.color.a = 1.0;
            marker_yellowSphere.color.r = 1.0;
            marker_yellowSphere.color.g = 1.0;
            marker_yellowSphere.color.b = 0.0;
            marker_yellowSphere.pose.orientation.w = 1.0;
            marker_yellowSphere.lifetime = ros::Duration();


            line_marker.header.frame_id = "world";  // Change this frame_id according to your setup
            line_marker.header.stamp = ros::Time::now();
            line_marker.ns = "line";
            line_marker.id = id_marker;
            id_marker++;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.scale.x = 0.05;  // Line width
            line_marker.pose.orientation.w = 1.0;
            line_marker.lifetime = ros::Duration();

            // Set the color (green in this case)
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;

            float gate_size = 0.75;

            //Generate the gate corners and edges
            Eigen::Matrix<double, 3, 1> move_gate;
            move_gate << 0.0, gate_size, gate_size;
            Eigen::Matrix<double, 3, 1> position2 = pos_gate + rotate_gate * move_gate;
            marker_yellowSphere.pose.position.x = position2(0);
            marker_yellowSphere.pose.position.y = position2(1);
            marker_yellowSphere.pose.position.z = position2(2);
            marker_yellowSphere.id = id_marker;
            id_marker++;
            line_marker.points.push_back(marker_yellowSphere.pose.position);
            marker_array.markers.push_back(marker_yellowSphere);

            move_gate << 0.0, -gate_size, gate_size;
            Eigen::Matrix<double, 3, 1> position = pos_gate + rotate_gate * move_gate;
            marker.pose.position.x = position(0);
            marker.pose.position.y = position(1);
            marker.pose.position.z = position(2);
            marker.id = id_marker;
            id_marker++;
            line_marker.points.push_back(marker.pose.position);
            marker_array.markers.push_back(marker);

            move_gate << 0.0, -gate_size, -gate_size;
            position = pos_gate + rotate_gate * move_gate;
            marker.pose.position.x = position(0);
            marker.pose.position.y = position(1);
            marker.pose.position.z = position(2);
            marker.id = id_marker;
            id_marker++;
            line_marker.points.push_back(marker.pose.position);
            marker_array.markers.push_back(marker);

            move_gate << 0.0, gate_size, -gate_size;
            position = pos_gate + rotate_gate * move_gate;
            marker_yellowSphere.pose.position.x = position(0);
            marker_yellowSphere.pose.position.y = position(1);
            marker_yellowSphere.pose.position.z = position(2);

            marker_yellowSphere.id = id_marker;
            id_marker++;
            marker_array.markers.push_back(marker_yellowSphere);
            line_marker.points.push_back(marker_yellowSphere.pose.position);

            marker.pose.position.x = position2(0);
            marker.pose.position.y = position2(1);
            marker.pose.position.z = position2(2);
            line_marker.points.push_back(marker.pose.position);
            marker_array.markers.push_back(line_marker);
            pub_gate_markers_.publish(marker_array);
        }

};

int main(int argc, char** argv) {


	ros::init(argc, argv, "move_drone");
	ros::NodeHandle nh("~");

    // Load the gates
    drone_race race;
    string filegates;
    filegates.assign("/home/nerea/AR/catkin_ws/src/arob_lab6/src/");
    if (argc>1){
        filegates.append(argv[argc-1]);
    }
    else {
        filegates.append("gates.txt");
    }
    race.readGates(filegates);

    ros::Rate loop_rate(10);
    for (int i = 0; i < 10; i++) {
        ros::spinOnce();  // process a few messages in the background - causes the uavPoseCallback to happen
        loop_rate.sleep();
    }
    race.drawGates();

    race.generate_trajectory_example();
    //race.generate_trajectory();

    while (ros::ok())
    {
        race.send_command();
        ros::spinOnce();
        loop_rate.sleep();
    }
	return 0;
}