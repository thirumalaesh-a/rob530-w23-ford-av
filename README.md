# 3D Particle Filter-based Localization and 2D Continuous CSM Mapping for the Ford AV Dataset
This project extends algorithmic work done for the [Ford Autonomous Vehicle dataset](https://avdata.ford.com/). Given the existing work in the Ford AV dataset, we proposed and implemented particle filter-based localization as an improvement to the EKF framework as well as a baseline mapping algorithm. Eventually, this work can enable full online SLAM in the Ford AV dataset.

For full results and discussion, see our final report that we submitted for ROB 530: Mobile Robotics in Winter 2023 at The University of Michigan.

## Localization
### Extended Kalman Filter


### Particle Filter

## Mapping
The baseline we implemented for mapping is a 2D [Continuous Counting Sensor Model](http://robots.stanford.edu/papers/haehnel.em-dynamic-icra03.pdf). An improvement would be to extend mapping to 3D and to use the state-of-the-art [BKI Mapping algorithm](https://github.com/ganlumomo/BKISemanticMapping).

### Continuous Counting Sensor Model
