#include <iostream>
#include <string>
#include <boost/timer.hpp>
#include <vector>
#include <numeric>

#include <ros/callback_queue.h>
#include <fstream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <boost/timer.hpp>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <tf/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

class Controller{

public :

    Controller(){
        init();
    }


    void init(){
        client_connect_to_ros();
        ros::Duration(1).sleep();

        // get the robot model
        robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
        robot_model_ = robot_model_loader.getModel();
//        // get the robot state
//        robot_state_ = robot_state::RobotStatePtr(new robot_state::RobotState(robot_model_));

        // define left and right arms joints home positions
        left_arm_joints_start_positions_ = {1.47, -0.67, -1.06, 1.42, 0.75, 1.24, 1.43};
        right_arm_joints_start_positions_ = {-1.52, -0.79, 1.17, 1.73, -0.74, 1.13, -1.21};

        // initializing left and right arms
        left_arm_group_.reset(new  moveit::planning_interface::MoveGroup(moveit::planning_interface::MoveGroup::Options("left_arm", moveit::planning_interface::MoveGroup::ROBOT_DESCRIPTION, nh)));
        right_arm_group_.reset(new moveit::planning_interface::MoveGroup(moveit::planning_interface::MoveGroup::Options("right_arm", moveit::planning_interface::MoveGroup::ROBOT_DESCRIPTION, nh)));

        // setting planner id for both arms
        left_arm_group_->setPlannerId(static_cast<std::string>(planner_params_["planner_id"]));
        right_arm_group_->setPlannerId(static_cast<std::string>(planner_params_["planner_id"]));

        // setting planning time for both arms
        left_arm_group_->setPlanningTime(std::stod(planner_params_["planning_time"]));
        right_arm_group_->setPlanningTime(std::stod(planner_params_["planning_time"]));

        // setting approach radius: approach distance before thouching the target
        approach_radius_ = std::stod(planner_params_["approach_radius"]);

    }

    void client_connect_to_ros(){
        nh = ros::NodeHandle("~");
        tf_subscriber_.reset(new ros::Subscriber(nh.subscribe("/tf", 10, &Controller::tfCallback, this)));
        planning_scene_pub_.reset(new ros::Publisher(nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1)));
        planning_scene_serv_.reset(new ros::ServiceClient(nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene", 1)));
        right_gripper_pub_ = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/right_gripper/command", true);
        left_gripper_pub_  = nh.advertise<baxter_core_msgs::EndEffectorCommand>("/robot/end_effector/left_gripper/command", true);

        // getting planner params
        nh.getParam(nh.getNamespace() + "/planner_parameters", planner_params_);
        nh.getParam("right_gripper_id", right_gripper_id_);
        nh.getParam("left_gripper_id", left_gripper_id_);
        asyn_spinner_.reset(new ros::AsyncSpinner(1));
        asyn_spinner_->start();
    }

    void tfCallback(const tf2_msgs::TFMessageConstPtr& msg){
        if(!goal1_.empty() && !goal1_normal_.empty()){
            static tf::TransformBroadcaster br;
            tf::Transform transform;

            //Set and publish a frame for the goal position
            transform.setOrigin( tf::Vector3(goal1_[0], goal1_[1], goal1_[2]) );
            tf::Quaternion q;
            q.setRPY(goal1_normal_[0], goal1_normal_[1], goal1_normal_[2]);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "goal1_frame"));


            //Set and publish a frame for all approach pose
            transform.setOrigin( tf::Vector3(approach_pose1_.pose.position.x, approach_pose1_.pose.position.y, approach_pose1_.pose.position.z) );
            q.setW(approach_pose1_.pose.orientation.w);
            q.setX(approach_pose1_.pose.orientation.x);
            q.setY(approach_pose1_.pose.orientation.y);
            q.setZ(approach_pose1_.pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "goal1_frame", "approach_frame1"));
            transform.setOrigin( tf::Vector3(goal1_pose_.pose.position.x, goal1_pose_.pose.position.y, goal1_pose_.pose.position.z) );
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "approach_frame1" , "final_goal1_frame"));

        }
	if(!goal2_.empty() && !goal2_normal_.empty()){
            static tf::TransformBroadcaster br;
            tf::Transform transform;

            //Set and publish a frame for the goal position
            transform.setOrigin( tf::Vector3(goal2_[0], goal2_[1], goal2_[2]) );
            tf::Quaternion q;
            q.setRPY(goal2_normal_[0], goal2_normal_[1], goal2_normal_[2]);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "goal2_frame"));


            //Set and publish a frame for all approach pose
            transform.setOrigin( tf::Vector3(approach_pose2_.pose.position.x, approach_pose2_.pose.position.y, approach_pose2_.pose.position.z) );
            q.setW(approach_pose2_.pose.orientation.w);
            q.setX(approach_pose2_.pose.orientation.x);
            q.setY(approach_pose2_.pose.orientation.y);
            q.setZ(approach_pose2_.pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "goal2_frame", "approach_frame2"));

            transform.setOrigin( tf::Vector3(goal2_pose_.pose.position.x, goal2_pose_.pose.position.y, goal2_pose_.pose.position.z) );
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "approach_frame2" , "final_goal2_frame"));

        }
	if(!goal3_.empty() && !goal3_normal_.empty()){
            static tf::TransformBroadcaster br;
            tf::Transform transform;

            //Set and publish a frame for the goal position
            transform.setOrigin( tf::Vector3(goal3_[0], goal3_[1], goal3_[2]) );
            tf::Quaternion q;
            q.setRPY(goal3_normal_[0], goal3_normal_[1], goal3_normal_[2]);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base", "goal3_frame"));


            //Set and publish a frame for all approach pose
            transform.setOrigin( tf::Vector3(approach_pose3_.pose.position.x, approach_pose3_.pose.position.y, approach_pose3_.pose.position.z) );
            q.setW(approach_pose3_.pose.orientation.w);
            q.setX(approach_pose3_.pose.orientation.x);
            q.setY(approach_pose3_.pose.orientation.y);
            q.setZ(approach_pose3_.pose.orientation.z);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "goal3_frame", "approach_frame3"));

            transform.setOrigin( tf::Vector3(goal3_pose_.pose.position.x, goal3_pose_.pose.position.y, goal3_pose_.pose.position.z) );
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "approach_frame3" , "final_goal3_frame"));

        }
    }

    bool goToStartingPosition(std::string arm_group_name){
        moveit::planning_interface::MoveGroup::Plan go_to_starting_position_plan;
        std::shared_ptr<moveit::planning_interface::MoveGroup> arm_group;
        std::vector<double> arm_joints_start_positions;
        bool in_start_position =false;
        // set the targeted arm joints group and its starting joints values
        if (strcmp(arm_group_name.c_str(),"left")) {
            arm_group = left_arm_group_;
            arm_joints_start_positions = left_arm_joints_start_positions_;
        }else{
            arm_group = right_arm_group_;
            arm_joints_start_positions = right_arm_joints_start_positions_;
        }
        // set the starting and targeted joints values for the left arm
        arm_group->setStartState(*left_arm_group_->getCurrentState());
        arm_group->setJointValueTarget(arm_joints_start_positions);
        // if the planning succeded, execute
        if (arm_group->plan(go_to_starting_position_plan)){
            ROS_INFO_STREAM("CONTROLLER:: The planner has found a plan to bring the left arm to starting position");
            in_start_position = static_cast<bool>(arm_group->execute(go_to_starting_position_plan));
        } else ROS_WARN_STREAM("CONTROLLER:: The planner has not found a plan to bring the left arm to starting position");

        if (in_start_position) ROS_INFO_STREAM("CONTROLLER:: "<< arm_group_name.c_str() << " arm in start position");
        else ROS_WARN_STREAM("CONTROLLER:: "<< arm_group_name.c_str() <<  " arm has FAILED to go to the starting position");
        return in_start_position;
    }


    //get largest difference between elements of two vectors
    double largestDifference(std::vector<double> &first, std::vector<double> &second){
        Eigen::VectorXd difference(first.size());
        double my_max = 0;
        for(size_t j = 0; j < first.size(); ++j)
            difference(j) = fabs(first[j] - second[j]);
        for(size_t j = 0; j < first.size(); ++j){
            if(difference(j) > my_max)
                my_max = difference(j);
        }
        return my_max;
    }

    void getApproachAndGoalPoses(){
        approach_pose1_ = getTargetPose("goal1_frame");
        goal1_pose_.pose.position.z = approach_radius_ + kExtentionDistance;
        approach_pose2_ = getTargetPose("goal2_frame");
        goal2_pose_.pose.position.z = approach_radius_ + kExtentionDistance;
        approach_pose3_ = getTargetPose("goal3_frame");
        goal3_pose_.pose.position.z = approach_radius_ + kExtentionDistance;
    }


  geometry_msgs::PoseStamped getTargetPose(std::string frame_id){
        Eigen::Vector3d approach_point;
        getApproachPoint(approach_point);
        // vector between the approach point(origin of the approach frame) and the goal(origin of the goal frame)
        tf::Vector3 V(-approach_point(0), -approach_point(1), -approach_point(2));
        V.normalize();
        // projection of the vector v in the xy plane of the approch frame
        tf::Vector3 VP(V.getX(), V.getY(), 0);
        // compute the rotations to make the z axis of the approach frame oriented towards the origin of the goal frame
        double yaw = atan2(V.getY(), V.getX());
        double pitch = atan2(VP.length() ,V.getZ());
        tf::Quaternion q;
        q.setRPY(0, pitch, yaw);

        geometry_msgs::PoseStamped approach_pose;
        approach_pose.header.frame_id = frame_id.c_str();
        approach_pose.pose.position.x = approach_point(0);
        approach_pose.pose.position.y = approach_point(1);
        approach_pose.pose.position.z = approach_point(2);
        approach_pose.pose.orientation.w = q.w();
        approach_pose.pose.orientation.x = q.x();
        approach_pose.pose.orientation.y = q.y();
        approach_pose.pose.orientation.z = q.z();
        return approach_pose;
    }


    // generate a random approach point around a part of a sphere: origin goal, radius: approach radius between -pi/3 and pi/3 in the goal frame
    void getApproachPoint(Eigen::Vector3d& approach_point){
        double theta = 0 ;//* u;
        double phi = 0;//* v;
        double x = approach_radius_*sin(phi)*cos(theta);
        double y = approach_radius_*sin(phi)*sin(theta);
        double z = approach_radius_*cos(phi);
        approach_point(0) = x;
        approach_point(1) = y;
        approach_point(2) = z;
    }

    // this function is used for enabling/disabling collision detection
    void octomap_manipulation(bool add_octomap_to_acm){

        moveit_msgs::GetPlanningScene::Request ps_req;
        moveit_msgs::GetPlanningScene::Response ps_res;
        moveit_msgs::PlanningScene ps_msg;


        ps_req.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;

        // get the planning scene response
        planning_scene_serv_->call(ps_req, ps_res);
        if(add_octomap_to_acm){
            ps_res.scene.allowed_collision_matrix.default_entry_names.push_back("<octomap>");
            ps_res.scene.allowed_collision_matrix.default_entry_values.push_back(true);
        }
        else{
            ps_res.scene.allowed_collision_matrix.default_entry_names.clear();
            ps_res.scene.allowed_collision_matrix.default_entry_values.clear();
        }

        // publish planning scene message
        ps_msg.is_diff = true;
        ps_msg.allowed_collision_matrix = ps_res.scene.allowed_collision_matrix;
        planning_scene_pub_->publish(ps_msg);
    }

    // reverse back a given trajectory of a given group
    bool reverseBackTrajectory(moveit::planning_interface::MoveGroup::Plan& traj_plan, std::string arm_group_name,
                               std::shared_ptr<moveit::planning_interface::MoveGroup> arm_group){

        moveit::planning_interface::MoveGroup::Plan reverse_plan;
        reverse_plan.trajectory_.joint_trajectory.header.stamp = ros::Time::now();
        reverse_plan.trajectory_.joint_trajectory.header.frame_id = traj_plan.trajectory_.joint_trajectory.header.frame_id;
        reverse_plan.trajectory_.joint_trajectory.joint_names = traj_plan.trajectory_.joint_trajectory.joint_names;

        int j = traj_plan.trajectory_.joint_trajectory.points.size() - 1;
        trajectory_processing::IterativeParabolicTimeParameterization time_param;
        robot_trajectory::RobotTrajectory robot_traj(robot_model_, arm_group_name);
        robot_state::RobotState r_state(robot_model_);
        for(size_t i = 0; i < traj_plan.trajectory_.joint_trajectory.points.size() && j >= 0; i++){
            moveit::core::jointTrajPointToRobotState(traj_plan.trajectory_.joint_trajectory, j, r_state);
            robot_traj.insertWayPoint(i, r_state, 0.1);
            j--;
        }

        if(!time_param.computeTimeStamps(robot_traj))
            ROS_WARN("TEST : Time parametrization for the solution path failed.");
        robot_traj.getRobotTrajectoryMsg(reverse_plan.trajectory_);
        moveit::core::robotStateToRobotStateMsg(*arm_group->getCurrentState(), reverse_plan.start_state_);
        return(static_cast<bool>(arm_group->execute(reverse_plan)));

    }


    //transform approach points and final goals to the planning frame "base"
    bool transformFrame(geometry_msgs::PoseStamped& output_transform,
                        std::string child_frame_name){

        tf::StampedTransform transform;
        std::string base_frame = "base";
        //make sure the goal frame is at the current goal child_frame_name;
        base_frame = "base";
        try{
            tf_listener_.lookupTransform(base_frame, child_frame_name, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }
        geometry_msgs::PoseStamped current_target;
        current_target.header.frame_id = base_frame;
        current_target.pose.position.x = transform.getOrigin().getX() ;
        current_target.pose.position.y = transform.getOrigin().getY() ;
        current_target.pose.position.z = transform.getOrigin().getZ() ;
        current_target.pose.orientation.w = transform.getRotation().getW();
        current_target.pose.orientation.x = transform.getRotation().getX();
        current_target.pose.orientation.y = transform.getRotation().getY();
        current_target.pose.orientation.z = transform.getRotation().getZ();
        output_transform =current_target;
        return true;
    }


    // Computes the distance between two std::vectors
    template <typename T>
    double  vectorsDistance(const std::vector<T>& a, const std::vector<T>& b)
    {
        std::vector<double> auxiliary;

        std::transform (a.begin(), a.end(), b.begin(), std::back_inserter(auxiliary),//
                        [](T element1, T element2) {return pow((element1-element2),2);});
        auxiliary.shrink_to_fit();

        return  sqrt(std::accumulate(auxiliary.begin(), auxiliary.end(), 0.0));
    } // end template vectorsDistance


    void calibrateGripper(ros::Publisher& gripper_pub, unsigned int gripper_id){
        baxter_core_msgs::EndEffectorCommand commandGripperMsg;

        commandGripperMsg.id = gripper_id;
        commandGripperMsg.command = "calibrate";
        gripper_pub.publish(commandGripperMsg);
    }


    void setGripperTopic(ros::Publisher& gripper_pub, unsigned int gripper_id){
        calibrateGripper(gripper_pub, gripper_id);
    }


    void openGripper(ros::Publisher& gripper_pub, unsigned int gripper_id){
        baxter_core_msgs::EndEffectorCommand commandGripperMsg;

        commandGripperMsg.id = gripper_id;
        commandGripperMsg.command = "go";
        commandGripperMsg.args = "{\"position\": 100.0}";
        gripper_pub.publish(commandGripperMsg);
    }


    void closeGripper(ros::Publisher& gripper_pub, unsigned int gripper_id){
        baxter_core_msgs::EndEffectorCommand commandGripperMsg;

        commandGripperMsg.id = gripper_id;
        commandGripperMsg.command = "go";
        commandGripperMsg.args = "{\"position\": 40.0}";
        gripper_pub.publish(commandGripperMsg);
    }


    bool rotateWrist(std::shared_ptr<moveit::planning_interface::MoveGroup> arm_group,
                     moveit::planning_interface::MoveGroup::Plan wrist_rotation_plan){

        //arm_group->setStartState(*arm_group->getCurrentState());
        std::vector< double >  joints_values = arm_group->getCurrentJointValues ();
        joints_values[joints_values.size()-1] =wrist_rot_array_[rand_rotation_value_index_];
        arm_group->setJointValueTarget(joints_values);

        return static_cast<bool>(arm_group->execute(wrist_rotation_plan));
    }


    double generateRandomRotationAngle(){
        int random_value = rand()%4;
        // generate random values until we get different value from the previously generated
        while(random_value == rand_rotation_value_index_){
            random_value =rand()%4;
        }
        rand_rotation_value_index_ =random_value;

        return wrist_rot_array_[rand_rotation_value_index_];
    }


    bool executeTrajectory(){

        std::shared_ptr<moveit::planning_interface::MoveGroup> arm_group;
        moveit::planning_interface::MoveGroup::Plan to_goal1_plan,to_goal3_plan, to_approach_point1_plan, to_approach_point2_plan, to_approach_point3_plan;
        robot_state::RobotStatePtr start_state_for_approach1_trajectory, start_state_for_approach2_trajectory,
	  start_state_for_approach3_trajectory,start_state_for_goal1_trajectory, start_state_for_goal3_trajectory;
        std::vector< double > arm_group_joints_state;
        std::string group_name, gripper_link_name;
        ros::Publisher gripper_pub;
        int gripper_id;
        geometry_msgs::Pose gripper_pose;
        bool to_approach_point1_plan_result = false;
        bool wrist_rotation_plan_result = false;

        ROS_INFO_STREAM("CONTROLLER:: Aquiring the targeted position of the  end effector");
        goal1_ = {0.4,0.6,0.2};
        goal1_normal_ = {0,0,0};
	goal2_ = {0.4,0.2,0.2};
        goal2_normal_ = {0,0,0};
        goal3_ = {0.6,0.2,0.2};
        goal3_normal_ = {0,0,0};
        int number_of_trials = 2;

        if(! goal_aquired_){
            goal_aquired_ = true;
            ROS_INFO_STREAM("CONTROLLER:: Getting random targets arround the sphere");
            getApproachAndGoalPoses();
        }

        //make sure that the octomap is in the collision world
        octomap_manipulation(true);

        // put left and right arms in the starting positions
        goToStartingPosition("left");
        goToStartingPosition("right");

        double fraction1, fraction2;
        double cartesian_path_success_threshold;
        int iteration = 0;
        bool complete_plan = false;
        tf::StampedTransform transform;
        while(!complete_plan && iteration < number_of_trials){
            //always plan for the approach with the octomap in the obstacle domain
            octomap_manipulation(false);

            std::vector<double> second1(3, 0);
            try{
                tf_listener_.lookupTransform("base", "goal1_frame", ros::Time(0), transform);
                second1 = {transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()};
                ros::Duration(1.0).sleep();
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            std::vector<double> vector1(3, 0);
            while((vectorsDistance(goal1_, vector1) - kExtentionDistance) > 0.01){
                //getApproachAndGoalPoses();

                //transform approach points and final goals to the planning frame "base"
                if(!transformFrame(transformed_approach_frame1_, "approach_frame1")){
		continue;
		ROS_INFO_STREAM("CONTROLLER::1 approach frame continue");
	      }
                if(!transformFrame(transformed_goal1_frame_, "final_goal1_frame")){
		continue;
		ROS_INFO_STREAM("CONTROLLER::1 final goal continue");}

                vector1 = {transformed_goal1_frame_.pose.position.x,
                          transformed_goal1_frame_.pose.position.y,
                          transformed_goal1_frame_.pose.position.z};

                ROS_INFO_STREAM("CONTROLLER::1- The distance between current goal and final goal frame is : " << vectorsDistance(goal1_, vector1));
                ROS_INFO("*************************************************");
                ros::Duration(1).sleep();
            }
	    std::vector<double> second2(3, 0);
            try{
                tf_listener_.lookupTransform("base", "goal2_frame", ros::Time(0), transform);
                second2 = {transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()};
                ros::Duration(1.0).sleep();
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            std::vector<double> vector2(3, 0);
            while((vectorsDistance(goal2_, vector2) - kExtentionDistance) > 0.01){
                //getApproachAndGoalPoses();

                //transform approach points and final goals to the planning frame "base"
                if(!transformFrame(transformed_approach_frame2_, "approach_frame2")){
		continue;
		ROS_INFO_STREAM("CONTROLLER::2 approach frame continue");
	      }
                    
                if(!transformFrame(transformed_goal2_frame_, "final_goal2_frame")){
		continue;
		ROS_INFO_STREAM("CONTROLLER::2 final goal continue");
		}

                vector2 = {transformed_goal2_frame_.pose.position.x,
                          transformed_goal2_frame_.pose.position.y,
                          transformed_goal2_frame_.pose.position.z};

                ROS_INFO_STREAM("CONTROLLER::2- The distance between current goal and final goal frame is : " << vectorsDistance(goal2_, vector2));
                ROS_INFO("*************************************************");
                ros::Duration(1).sleep();
            }
	    std::vector<double> second3(3, 0);
	    try{
                tf_listener_.lookupTransform("base", "goal3_frame", ros::Time(0), transform);
                second3 = {transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ()};
                ros::Duration(1.0).sleep();
            }
            catch (tf::TransformException &ex) {
                ROS_ERROR("%s",ex.what());
                ros::Duration(1.0).sleep();
            }

            std::vector<double> vector3(3, 0);
            while((vectorsDistance(goal3_, vector3) - kExtentionDistance) > 0.01){
                //getApproachAndGoalPoses();

                //transform approach points and final goals to the planning frame "base"
	      if(!transformFrame(transformed_approach_frame3_, "approach_frame3")){
		continue;
		ROS_INFO_STREAM("CONTROLLER::3 approach frame continue");
	      }
	      if(!transformFrame(transformed_goal3_frame_, "final_goal3_frame")){
		continue;
		ROS_INFO_STREAM("CONTROLLER::3 final goal continue");
	      }

                vector3 = {transformed_goal3_frame_.pose.position.x,
                          transformed_goal3_frame_.pose.position.y,
                          transformed_goal3_frame_.pose.position.z};

                ROS_INFO_STREAM("CONTROLLER::3- The distance between current goal and final goal frame is : " << vectorsDistance(goal3_, vector3));
                ROS_INFO("*************************************************");
                ros::Duration(1).sleep();
            }
            // set the targeted arm joints group and its starting joints values
            if(goal1_[1] >= 0){
                group_name = "left_arm";
                arm_group = left_arm_group_;
                gripper_id = left_gripper_id_;
                gripper_link_name = "left_gripper";
                gripper_pub = left_gripper_pub_;
            }else{
                group_name = "right_arm";
                arm_group = right_arm_group_;
                gripper_id = right_gripper_id_;
                gripper_link_name = "right_gripper";
                gripper_pub = right_gripper_pub_;
            }

            // plan the first part of the trajectory: from the starting position to the approach point
            ROS_INFO_STREAM("CONTROLLER:: Planning a trajectory to the selected approach point");
            arm_group->setStartState(*arm_group->getCurrentState());
            arm_group->setPoseTarget(transformed_approach_frame1_);
            to_approach_point1_plan_result = static_cast<bool>(arm_group->plan(to_approach_point1_plan));


            if (to_approach_point1_plan_result){
	      start_state_for_goal1_trajectory = arm_group->getCurrentState();
	      ROS_INFO_STREAM("CONTROLLER:: Planning a trajectory to the specified goal");
	      moveit::core::jointTrajPointToRobotState(to_approach_point1_plan.trajectory_.joint_trajectory,
						       to_approach_point1_plan.trajectory_.joint_trajectory.points.size() - 1,
						       *start_state_for_goal1_trajectory);
	      octomap_manipulation(true);
	      std::vector<geometry_msgs::Pose> waypoints1;
	      waypoints1.push_back(transformed_approach_frame1_.pose);
	      waypoints1.push_back(transformed_goal1_frame_.pose);

	      moveit_msgs::RobotTrajectory robot_trajectory1;
	      arm_group->setStartState(*start_state_for_goal1_trajectory);
	      fraction1 = arm_group->computeCartesianPath(waypoints1, 0.01, 0.0, robot_trajectory1);
	      
	      // compute the success threshold: a computed cartesian path is considered a success
	      //if it return a faction greater than this threshold
	      //cartesian_path_success_threshold = approach_radius_/(- kExtentionDistance + approach_radius_ );
	      cartesian_path_success_threshold = (approach_radius_ - kExtentionDistance)/approach_radius_ ;
	      if(fraction1 >= cartesian_path_success_threshold){
		to_goal1_plan.trajectory_ = robot_trajectory1;
		start_state_for_approach2_trajectory = arm_group->getCurrentState();
		ROS_INFO_STREAM("CONTROLLER:: Planning a trajectory to the specified goal");
		moveit::core::jointTrajPointToRobotState(to_goal1_plan.trajectory_.joint_trajectory,
							 to_goal1_plan.trajectory_.joint_trajectory.points.size() - 1,
							 *start_state_for_approach2_trajectory);
		
		std::vector<geometry_msgs::Pose> waypoints2;
		waypoints2.push_back(transformed_goal1_frame_.pose);
		waypoints2.push_back(transformed_goal2_frame_.pose);
		waypoints2.push_back(transformed_goal3_frame_.pose);
		
		moveit_msgs::RobotTrajectory robot_trajectory2;
		arm_group->setStartState(*start_state_for_approach2_trajectory);
		fraction2 = arm_group->computeCartesianPath(waypoints2, 0.01, 0.0, robot_trajectory2);
		      
		cartesian_path_success_threshold = (approach_radius_ - kExtentionDistance)/approach_radius_ ;
		ROS_INFO_STREAM("CONTROLLER:: cartesian_path "<<fraction1);
		if (fraction2 > 0){
		  to_approach_point2_plan.trajectory_ = robot_trajectory2;
		  ROS_INFO_STREAM("CONTROLLER:: Executing planned motion");
		    // Execute plane: from starting position to the approach position
		    bool to_approach_point_motion_result = static_cast<bool>(arm_group->execute(to_approach_point1_plan));
		    ros::Duration(1).sleep();		    // open gripper
		    openGripper(gripper_pub,gripper_id);
		    ros::Duration(1).sleep();
		    // Execute plane: from approach position to the specified goal
		    moveit::core::robotStateToRobotStateMsg(*arm_group->getCurrentState(), to_goal1_plan.start_state_);
		    //arm_group->setStartState(*arm_group->getCurrentState());
		    bool to_goal_motion1_result = static_cast<bool>(arm_group->execute(to_goal1_plan));
		    // Execute plane: from approach position to the specified goal
		    moveit::core::robotStateToRobotStateMsg(*arm_group->getCurrentState(), to_approach_point2_plan.start_state_);
		    // close gripper
		    closeGripper(gripper_pub,gripper_id);
		    ros::Duration(1).sleep();
		    //arm_group->setStartState(*arm_group->getCurrentState());
		    bool to_goal_motion2_result = static_cast<bool>(arm_group->execute(to_approach_point2_plan));
		    if (to_goal_motion2_result){
		      // open gripper
		      openGripper(gripper_pub,gripper_id);
		      ros::Duration(1).sleep();
		      reverseBackTrajectory(to_approach_point2_plan, group_name, arm_group);
		      // reverse back motion from starting position to the approach position
		      reverseBackTrajectory(to_goal1_plan, group_name, arm_group);
		      // reverse back motion from starting position to the approach position
		      reverseBackTrajectory(to_approach_point1_plan, group_name, arm_group);
		    }else {ROS_WARN_STREAM("CONTROLLER:: Failed to execute waypoints");}
		}else{ ROS_WARN_STREAM("CONTROLLER:: Failed to find a straight line to goal");}
	      }else{ ROS_WARN_STREAM("CONTROLLER:: Failed to find a straight line to waypoints");}
	    } else ROS_WARN_STREAM("CONTROLLER:: Failed to find a plan from the starting position to the approach point");
	}
        goal1_.clear();
        return ((fraction1 >= cartesian_path_success_threshold) && to_approach_point1_plan_result);
    }


    void client_disconnect_from_ros(){}
    void update(){}
    ros::NodeHandle nh;

private:
  std::vector<double> goal1_, goal1_normal_,goal2_, goal2_normal_,goal3_, goal3_normal_;
    XmlRpc::XmlRpcValue planner_params_;
    std::shared_ptr<moveit::planning_interface::MoveGroup> left_arm_group_, right_arm_group_;
    //bool left_arm_in_start_position_,right_arm_in_start_position_;
    std::unique_ptr<ros::AsyncSpinner> asyn_spinner_;
    const double wrist_rot_array_[4] = {0, M_PI/4, M_PI/2, 3*M_PI/4};
    //int  _wristRotArrayIndex=0;
    moveit::planning_interface::MoveGroup::Plan _wristRotationPlan;
    std::map<std::string, double> _turn_variable_values;
    int rand_rotation_value_index_ = rand() % 4;
    ros::Publisher  right_gripper_pub_, left_gripper_pub_;
    robot_state::RobotStatePtr robot_state_;

    int right_gripper_id_;
    int left_gripper_id_;

    std::unique_ptr<ros::ServiceClient> planning_scene_serv_;
    std::unique_ptr<ros::Publisher> planning_scene_pub_;
    std::shared_ptr<ros::Subscriber> sub_joint_states_, tf_subscriber_;
    robot_model::RobotModelPtr robot_model_;
    std::unique_ptr<ros::ServiceClient> move_baxter_arm_;


    geometry_msgs::PoseStamped transformed_approach_frame1_,transformed_approach_frame2_,transformed_approach_frame3_,
      approach_pose1_,approach_pose2_,approach_pose3_,
      goal1_pose_,goal2_pose_,goal3_pose_,
      transformed_goal1_frame_,transformed_goal2_frame_,transformed_goal3_frame_;

    tf::TransformListener tf_listener_;

    bool goal_aquired_ = false;
    std::vector<double> left_arm_joints_start_positions_,
    right_arm_joints_start_positions_;
    const double kExtentionDistance = 0.05;
    const double  kRetractDistance = 0.2;
    double approach_radius_;
};

std::string parse_arg(int& argc, char **& argv, const std::string& default_val)
{
    std::string key;
    std::string value;
    std::string temp_str;
    std::string::size_type res;

    key = "__name:=";
    for (unsigned short i = 0; i < argc; ++i) {
        temp_str = argv[i];
        res = temp_str.find(key);

        if (res != std::string::npos) {
            value = temp_str.erase(res, key.length());
            break;
        }
        else if (i == argc - 1) {
            value = default_val;
        }
    }
    return value;
}

int main(int argc, char** argv){
    std::string node_name;
    node_name = parse_arg(argc, argv, "push_button_primitive_node");
    ros::init(argc, argv, node_name);

    Controller controller;

    ROS_INFO_STREAM("CONTROLLER:: NameSpace: "<<controller.nh.getNamespace());

    ROS_INFO_STREAM("CONTROLLER:: Robot controller ready !");
    while (ros::ok()) {
        controller.executeTrajectory();
        usleep(1000);
        ros::spinOnce();
    }
    return 0;
}
