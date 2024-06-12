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

void SaveWaypoint(std::ofstream &f, ros::Time timestamp, Eigen::Vector3d desired_position, double desired_yaw){
  f << std::setprecision(6) << timestamp << ","
    << desired_position.x() << "," << desired_position.y() << "," << desired_position.z() << "," 
    << desired_yaw << std::endl;
    
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "rectangular_waypoint_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  
  ROS_INFO("Started rectangular example.");

  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;

  // Trying to unpause Gazebo for 10 seconds.
  while (i <= 10 && !unpaused) {
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
  // control command 10hz
  ros::Rate rate(10);
  // Default desired position and yaw.
  Eigen::Vector3d desired_position(10.0, 5.0, 12.0);
  double desired_yaw = 0.0;

  // open filestream
  std::string traj_filename;
  nh_private.param<std::string>("traj_filename", traj_filename, "/home/nmsl/rotorS_ws/sim_trajectory.csv");
  std::ofstream f;
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
  nh_private.param("yaw", desired_yaw, desired_yaw);
  const float DEG_2_RAD = M_PI / 180.0;
  int orientation = 1;

  float facing = 90;
  int rot = 30;
  int t = 30;
  bool takeoff = 1;
  while(ros::ok()){
    if(takeoff){
      trajectory_msg.header.stamp = ros::Time::now();
      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
      desired_position, desired_yaw, &trajectory_msg);
      trajectory_pub.publish(trajectory_msg);
      SaveWaypoint(f, trajectory_msg.header.stamp,desired_position,desired_yaw);
      ros::Duration(5).sleep(); //wait 5 sec for takeoff
      takeoff = 0;
    }

    if(desired_position.x() > 30 || desired_position.x() < 15 || desired_position.y() > 15 || desired_position.y() < 5){
      switch(orientation){
        case 1:
          if(desired_position.x() > 30){
            orientation++;
            t = 1;
          }
          break;
        case 2:
          if(desired_position.y() > 15){
            orientation++;
            t = 1;
          }
          break;
        case 3:
          if(desired_position.x() < 15){
            orientation++;
            t = 1;
          }
          break;
        case 4:
          if(desired_position.y() < 5){
            orientation = 1;
            t = 1;
          }
          break;
      }
    }
    

    int unit = facing / rot;

    switch(orientation){
      case 1:
        if(rot > t){
          desired_yaw = (0.0 + unit * t) * DEG_2_RAD;
          t++;
        }
        else{
          desired_yaw = ((0.0 + facing) * DEG_2_RAD);
        }
        
        desired_position.x() = desired_position.x() + 0.1; 
      break;
      case 2:
        if(rot > t){
          desired_yaw = (90.0 + unit * t) * DEG_2_RAD;
          t++;
        }
        else{
          desired_yaw = ((90.0 + facing) * DEG_2_RAD);
        }
        desired_position.y() = desired_position.y() + 0.1; 
      break;
      case 3:
        if(rot > t){
          desired_yaw = (180.0 + unit * t) * DEG_2_RAD;
          t++;
        }
        else{
          desired_yaw = ((180.0 + facing) * DEG_2_RAD);
        }
        desired_position.x() = desired_position.x() - 0.1; 
      break;
      case 4:
        if(rot > t){
          desired_yaw = (270.0 + unit * t) * DEG_2_RAD;
          t++;
        }
        else{
          desired_yaw = ((270.0 + facing) * DEG_2_RAD);
        }
        
        desired_position.y() = desired_position.y() - 0.1; 
      break;
    }
    ROS_INFO("orientation: %d %lf", orientation, desired_yaw);
    //desired_position.z() = 1;
    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
    desired_position, desired_yaw, &trajectory_msg);    
    
    //cyw dump traj as csv file

    SaveWaypoint(f, trajectory_msg.header.stamp, desired_position, desired_yaw);

    trajectory_pub.publish(trajectory_msg);
    rate.sleep();
    ros::spinOnce();

  }
  std::cout << "Saving waypoint to "<< traj_filename << std::endl;
  f.close();
  ros::shutdown();

  return 0;
}
