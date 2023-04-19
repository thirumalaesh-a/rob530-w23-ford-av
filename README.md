# ROB 530 Team 23: Particle Filter-Based Localization and Continuous CSM Mapping for the Ford AV Dataset
This project extends algorithmic work done for the [Ford Autonomous Vehicle dataset](https://avdata.ford.com/). Given the existing work in the Ford AV dataset, we proposed and implemented particle filter-based localization as an improvement to the EKF framework as well as a baseline mapping algorithm. Eventually, this work can enable full online SLAM in the Ford AV dataset.

For full results and discussion, see our final report that we submitted for ROB 530: Mobile Robotics in Winter 2023 at The University of Michigan.

## Localization

### Particle Filter
The particle filter can be found in the `particle_filter` folder. Simply run `run.py` to see the filtered poses and raw poses get written out to text files.

### Extended Kalman Filter
Our particle filter results did not yield results with the expected improvement due to the algorithm's time complexity and particle degeneracy with a higher number of particles. We applied the extended kalman filter from the `robot_localization` node and compared the translational and orientation errors with our particle filter. With the EKF, we achieved better results. The EKF with its results can be found in the `ekf` folder.

## Mapping
The baseline we implemented for mapping is a 2D [Continuous Counting Sensor Model](http://robots.stanford.edu/papers/haehnel.em-dynamic-icra03.pdf). An improvement would be to extend mapping to 3D and to use the state-of-the-art [BKI Mapping algorithm](https://github.com/ganlumomo/BKISemanticMapping).

### Continuous Counting Sensor Model
The model can be found in the `mapping` folder.
