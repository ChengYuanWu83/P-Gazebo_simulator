# P-Gazebo
we present P-Gazebo, the first UAV simulator based on 3D Gaussian Splatting (3DGS) representation. 
Instead of using 3D mesh or 3D point cloud generated from scanning, utilizing 3DGS as the representation allows users to reconstruct the scene from a set of RGB images, while simultaneously maintaining a high level of image quality.
Our simulator can provide photo-realistic rendering, realistic physics simulation, and a rich array of sensors that can be employed simultaneously. 
## Problem Statement
Without the help of the simulator, whenever the researchers need to test their algorithm, such as object tracking, they have to collect the dataset from the real-world and elaborately set the sensors. 
The process may be expensive and complex. However, utilizing the UAV simulator for testing before transitioning to the actual UAVs can significantly reduce cost, and be able to collect data faster, which could increase the iteration of the algorithms.
Some existing UAV simulators are usually used for UAV application development and experiments, such as AirSim, Gazebo and FlightGoggles.
Airsim provides photo-realistic visual environments for testing and developing algorithms related to computer vision.
Gazebo provides variable sensor and UAV models.
FlightGoggles provides a custom UAV physics model not under the physical engine and photo-realistic renderer.
Three things are important for the UAV simulator, which are the complicated UAV dynamics model, variable sensor, and photorealistic scene.
However, the existing simulators can't deal with all three things well simultaneously.

## Solution Approach
To address the aforementioned issues, we proposed a P-Gazebo. 
The architecture of P-Gazebo is divided into a physics engine, a rendering engine, and an interface between the them. 
The physics engine simulates the dynamics of the UAV and the interactions with the environment, such as gravity, wind fields, and collisions. Various sensors also can be deployed on the UAV to collect data about the environment.
The rendering engine renders the scenes of the 3D environment photo-realistic scene which reconstructed by 3D Gaussian Splatting and captures images of the current pose of the UAV.
The interface in between is responsible for transmitting the pose of the UAV and images between the two engines.
This project builds on the open-source project: [RotorS](https://github.com/ethz-asl/rotors_simulator)

## Conclusion
Our main contributions is that we offer a simulator that enable researcher to collect 6 DoF photo-realistic image and UAV dynamics features simultaneously.
Using simulator to collect data can easily collect large amounts of data and test the UAV algorithm in the photo-realistic environment.
