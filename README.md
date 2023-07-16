# ROB 530 Team 23: Particle Filter-Based Localization and Continuous CSM Mapping for the Ford AV Dataset
# Authors: Thirumalaesh Ashokkumar, Isha Bhatt, Yipeng Lin, Meghana Kowsika, Yanjun Chen  
This project extends algorithmic work done for the [Ford Autonomous Vehicle dataset](https://avdata.ford.com/). Given the existing work in the Ford AV dataset, we proposed and implemented particle filter-based localization as an improvement to the EKF framework as well as a baseline mapping algorithm. Eventually, this work can enable full online SLAM in the Ford AV dataset.

For full results and discussion, see our [final presentation slides](https://docs.google.com/presentation/d/1TLR9U1n9wFX8QCuJQftZjpha_fBKxPrHPW6eUaHERZc/edit#slide=id.g2188f2d8693_0_0) for ROB 530: Mobile Robotics in Winter 2023 at The University of Michigan.

## Localization

### Particle Filter
The particle filter can be found in the `particle_filter` folder. Simply run `run.py` to see the filtered poses and raw poses get written out to text files.

### Extended Kalman Filter
Our particle filter results did not yield results with the expected improvement due to the algorithm's time complexity and particle degeneracy with a higher number of particles. We applied the extended kalman filter from the `robot_localization` node and compared the translational and orientation errors with our particle filter. With the EKF, we achieved better results. The EKF with its results can be found in the `ekf` folder.

## Mapping
The baseline we implemented for mapping is a 2D [Continuous Counting Sensor Model](http://robots.stanford.edu/papers/haehnel.em-dynamic-icra03.pdf). An improvement would be to extend mapping to 3D and to use the state-of-the-art [BKI Mapping algorithm](https://github.com/ganlumomo/BKISemanticMapping).

### Continuous Counting Sensor Model
The model can be found in the `mapping` folder.

First, download the Ford AV [sample dataset](https://github.com/Ford/AVData#dataset-download) and specify the appropriate directory for the Rosbag in `mapping/config/settings.yaml`

To run the mapping algorithm, run the following scripts in this order:
* `rosbag_lidar_to_pickle.py`
* `rosbag_rawpose_to_pickle.py`
* `align_lidar_rawpose.py`
* `run.py`

## System Requirements
This repository has been tested on on Ubuntu 16.04, Ubuntu 18.04, and Ubuntu 20.04 with ROS Noetic.

## Library Dependencies
* rospy
* rosbag
* pickle
