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
  ros::init(argc, argv, "circle_waypoint_example");
  ros::NodeHandle nh;
  // Create a private node handle for accessing node parameters.
  ros::NodeHandle nh_private("~");
  ros::Publisher trajectory_pub =
      nh.advertise<trajectory_msgs::MultiDOFJointTrajectory>(
          mav_msgs::default_topics::COMMAND_TRAJECTORY, 10);
  ROS_INFO("Started circle example.");

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
  double t = 0;
  double angle = 0;
  ros::Rate rate(10);
  // Default desired position and yaw.





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
  bool takeoff = 1 ;
  double set_x = 0.0;
  double set_y = 0.0;
  double set_z = 12.0;
  float scale = 4;  
  double desired_yaw = 0.0;
  // Overwrite defaults if set as node parameters.
  nh_private.param("x", set_x, 0.0);
  nh_private.param("y", set_y, 0.0);
  nh_private.param("z", set_z, 12.0);

  Eigen::Vector3d desired_position(set_x, set_y, set_z);

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
    t+=0.01;
    angle = fmod(2 * M_PI / 10 * t, 2 * M_PI);
    desired_position.x() = cos(angle)* scale + set_x;
    desired_position.y() = sin(angle)* scale + set_y;
    //desired_position.z() = 1;
    desired_yaw = fmod(angle + M_PI, 2 * M_PI);
  

    trajectory_msg.header.stamp = ros::Time::now();
    mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
    desired_position, desired_yaw, &trajectory_msg);
    trajectory_pub.publish(trajectory_msg);
    rate.sleep();
    ros::spinOnce();

    //cyw dump traj as csv file

    SaveWaypoint(f, trajectory_msg.header.stamp, desired_position, desired_yaw);

    
  }
  std::cout << "Saving waypoint to "<< traj_filename << std::endl;
  f.close();
  ros::shutdown();

  return 0;
}
