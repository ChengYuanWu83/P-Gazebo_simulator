/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <thread>
#include <chrono>
#include <string>
#include <iostream>
#include <fstream>


#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Bool.h>

ros::Subscriber position_cmd_sub, finish_sub;
ros::Publisher trajectory_pub;
std::ofstream f;
bool plannerIsPlanning = 0;

Eigen::Vector3d desired_position(0.0, 0.0, 3.0);
double desired_yaw = 0.0;
//record cmd to avoid return back
double x_low = desired_position.x();
double x_high = x_low;

void cmdCallback(const quadrotor_msgs::PositionCommand::ConstPtr& cmd){
  plannerIsPlanning = 1;  
  
  // x_low = std::min(cmd->position.x, x_low);
  // x_high = std::max(cmd->position.x, x_high);
  // if(cmd->position.x > x_low && cmd->position.x < x_high){
  //   return;
  // }

  float scale_factor = 1.0;
  desired_position =  Eigen::Vector3d(cmd->position.x * scale_factor, cmd->position.y * scale_factor, desired_position.z());
  //desired_position =  Eigen::Vector3d(cmd->position.x * scale_factor, cmd->position.y * scale_factor, cmd->position.z * scale_factor);
  desired_yaw = cmd->yaw;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();
  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
  desired_position, desired_yaw, &trajectory_msg);
  trajectory_pub.publish(trajectory_msg);

  f << std::setprecision(6) << trajectory_msg.header.stamp << ","
  << desired_position.x() << "," << desired_position.y() << "," << desired_position.z() << "," 
  <<  desired_yaw << std::endl;
}
void finishCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (msg->data == true) {
    ROS_INFO("Shutting Down.");
    ros::shutdown();
  }

}
int main(int argc, char** argv) {
  ros::init(argc, argv, "path_planner_publisher");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  position_cmd_sub = nh.subscribe("/drone_0_planning/pos_cmd", 50, &cmdCallback);
  finish_sub = nh.subscribe("/drone_0_planning/finish", 50, &finishCallback);
  trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started path planner publisher.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 15 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }

  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return -1;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
  }

  // Wait for 5 seconds to let the Gazebo GUI show up.
  ros::Duration(5.0).sleep();

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  ros::Rate rate(100);
  // Default desired position and yaw.


  // open filestream
  std::string traj_filename;
  nh_private.param<std::string>("traj_filename", traj_filename, "/home/nmsl/rotorS_ws/sim_trajectory.csv");
  f.open(traj_filename.c_str());
  if (!f.is_open()) {
    std::cout << "cannot open " << traj_filename<< std::endl;
  }else{
    std::cout << "Writing waypoint to "<< traj_filename << std::endl;
  }
  f << std::fixed;
  // label:timestamp,x,y,z,yaw
  f << "timestamp,x,y,z,yaw" << std::endl;

  // Overwrite defaults if set as node parameters.
  nh_private.param("x", desired_position.x(), desired_position.x());
  nh_private.param("y", desired_position.y(), desired_position.y());
  nh_private.param("z", desired_position.z(), desired_position.z());

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);

  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh.getNamespace().c_str(), desired_position.x(),
           desired_position.y(), desired_position.z());
  trajectory_pub.publish(trajectory_msg);

  // Wait for 1 seconds to let the UAV takeoff.
  while(ros::ok()){
    if(!plannerIsPlanning){
      desired_position.x() = desired_position.x() + 0.01;

      trajectory_msg.header.stamp = ros::Time::now();
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
      trajectory_pub.publish(trajectory_msg);
      //cyw: dump traj as csv file
      f << std::setprecision(6) << trajectory_msg.header.stamp << ","
      << desired_position.x() << "," << desired_position.y() << "," << desired_position.z() << "," 
      <<  desired_yaw << std::endl;
    }

    rate.sleep();
    ros::spinOnce();
  }

  ros::shutdown();

  std::cout << "Saving waypoint to "<< traj_filename << std::endl;
  f.close();
  return 0;
}
