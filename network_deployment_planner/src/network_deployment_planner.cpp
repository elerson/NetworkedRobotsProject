/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Authors: Elerson Rubens da Silva Santos (elerss<at>gmail.com)
*********************************************************************/
#include <network_deployment_planner/network_deployment_planner.h>
#include <pluginlib/class_list_macros.h>
#include <visualization_msgs/Marker.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(network_deployment_planner::NetworkDeploymentPlanner, nav_core::BaseGlobalPlanner)

namespace network_deployment_planner {

  NetworkDeploymentPlanner::NetworkDeploymentPlanner()
  : costmap_ros_(NULL), initialized_(false){}

  NetworkDeploymentPlanner::NetworkDeploymentPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), initialized_(false){
    initialize(name, costmap_ros);
  }
  
  void NetworkDeploymentPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    if(!initialized_){
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap();
      ROS_INFO("nome %s",name.c_str());
      ros::NodeHandle private_nh("~");
      ROS_INFO("namespace %s",private_nh.getNamespace().c_str());
      private_nh.param(private_nh.getNamespace()+"/step_size", step_size_, costmap_->getResolution());
      resolution_ = costmap_->getResolution();
      private_nh.param(private_nh.getNamespace()+"/min_dist_from_robot", min_dist_from_robot_, 0.10);
      private_nh.param<std::string>(private_nh.getNamespace()+"/planner_file_1", planner_file_1_, "filedata1.dat");
      private_nh.param<std::string>(private_nh.getNamespace()+"/planner_file_2", planner_file_2_, "filedata2.dat");
      private_nh.param<std::string>(private_nh.getNamespace()+"/planner_file_dir", planner_file_dir_, "/tmp/");
      ROS_INFO("nome %s",planner_file_dir_.c_str());
      ROS_INFO("nome %f",resolution_);

      tree_representation_.setMapSize(costmap_->getSizeInMetersX(), costmap_->getSizeInMetersY());
      tree_representation_.setResolution(resolution_);
      tree_representation_.setDistanceBetweenPoints(0.1);
	    tree_representation_.load(planner_file_dir_, planner_file_1_, planner_file_2_);

      world_model_ = new base_local_planner::CostmapModel(*costmap_);
      initialized_ = true;

    }
    else
      ROS_WARN("This planner has already been initialized... doing nothing");
  }

  //we need to take the footprint of the robot into account when we calculate cost to obstacles
  double NetworkDeploymentPlanner::footprintCost(double x_i, double y_i, double theta_i){
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return -1.0;
    }

    std::vector<geometry_msgs::Point> footprint = costmap_ros_->getRobotFootprint();
    //if we have no footprint... do nothing
    if(footprint.size() < 3)
      return -1.0;

    //check if the footprint is legal
    double footprint_cost = world_model_->footprintCost(x_i, y_i, theta_i, footprint);
    return footprint_cost;
  }



  bool NetworkDeploymentPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
  	ROS_INFO("making plans");
    if(!initialized_){
      ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
      return false;
    }

    ROS_INFO("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

    plan.clear();
    costmap_ = costmap_ros_->getCostmap();

    if(goal.header.frame_id != costmap_ros_->getGlobalFrameID()){
      ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
          costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
      return false;
    }

    tf::Stamped<tf::Pose> goal_tf;
    tf::Stamped<tf::Pose> start_tf;

    poseStampedMsgToTF(goal,goal_tf);
    poseStampedMsgToTF(start,start_tf);

    double useless_pitch, useless_roll, goal_yaw, start_yaw;
    start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
    goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

    //we want to step back along the vector created by the robot's position and the goal pose until we find a legal cell
    double goal_x = goal.pose.position.x;
    double goal_y = goal.pose.position.y;
    double start_x = start.pose.position.x;
    double start_y = start.pose.position.y;

    double diff_x = goal_x - start_x;
    double diff_y = goal_y - start_y;
    double diff_yaw = angles::normalize_angle(goal_yaw-start_yaw);
   

    std::vector<point2d_> path;
    path.clear();
    //ROS_INFO("start and goal plan(%f)(%f)(%f)(%f)",start_x, start_y, goal_x, goal_y);
    tree_representation_.findPath(start_x, start_y, goal_x, goal_y, path);
    ROS_INFO("start and goal plan(%f)(%f)(%f)(%f), size %d",start_x, start_y, goal_x, goal_y, path.size());
    for(int i = path.size()-1; i > 0 ; --i){
    	geometry_msgs::PoseStamped new_goal = goal;
    	new_goal.pose.position.x = path.at(i).x;
     	new_goal.pose.position.y = path.at(i).y;
 
 		  //define the quaternion representing the direction of the goal
 		  tf::Quaternion goal_quat = tf::createQuaternionFromYaw(path.at(i).angle);
    	new_goal.pose.orientation.x = goal_quat.x();
    	new_goal.pose.orientation.y = goal_quat.y();
    	new_goal.pose.orientation.z = goal_quat.z();
    	new_goal.pose.orientation.w = goal_quat.w();

   		plan.push_back(new_goal);

    }
    ROS_INFO("start plan(%f)(%f)",start.pose.position.x, start.pose.position.y);
    ROS_INFO("goal plan(%f)(%f)(%d)",goal.pose.position.x, goal.pose.position.y,  plan.size());
    for(int i = path.size() -1; i > 0 ; i = i - 90)
    	ROS_INFO("plan %f,%f", path.at(i).x, path.at(i).y);
	  int mid = 0;
    //while(ros::ok())
    /*{
    	ros::NodeHandle n;
    	ros::Publisher vis_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
		std::map<int, point2d_>::iterator vertice_it;
		for(int y = 0; y < 4; y++){
			for(vertice_it = tree_representation_.getVertices().begin(); vertice_it != tree_representation_.getVertices().end(); ++vertice_it){
			    visualization_msgs::Marker marker;
				marker.header.frame_id = "map";
				marker.header.stamp = ros::Time();
				marker.ns = "my_namespace";
				marker.id = mid++;
				marker.type = visualization_msgs::Marker::SPHERE;
				marker.action = visualization_msgs::Marker::ADD;
				marker.pose.position.x = vertice_it->second.x;
				marker.pose.position.y = vertice_it->second.y;
				marker.pose.position.z = 0;
				marker.pose.orientation.x = 0.0;
				marker.pose.orientation.y = 0.0;
				marker.pose.orientation.z = 0.0;
				marker.pose.orientation.w = 1.0;
				marker.scale.x = 1;
				marker.scale.y = 1;
				marker.scale.z = 1;
				marker.color.a = 1.0;
				marker.color.r = 0.0;
				marker.color.g = 1.0;
				marker.color.b = 0.0;

				vis_pub.publish( marker );
				//ROS_INFO("(%d)(%f,%f)",marker.id, marker.pose.position.x,marker.pose.position.y);

				
				ros::spinOnce();
			}
		sleep(1);
	}

		
	}*/


    return true; 
    
  }

};
