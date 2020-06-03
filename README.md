# SLAMBot
<sup>NOTE: the Robotic Systems Laboratory course refers to this robot as an MBot, but I will refer to it as SLAMBot for the sake of clarity about its functionality.</sup>

The SLAMBot project is one of three projects I took part in for the Robotic Systems Laboratory course (ROB 550) at the University of Michigan. The goal was to use C++ program a LiDAR-equipped two-wheeled robot to map out and navigate a maze environment given no prior information. Due to the COVID-19 outbreak, the project moved from an in-person group project to an individual one done via simulation, so all implementation was done by me.

## Part 1: Simultaneous Localization & Mapping (SLAM)
The first part of the project involved the implementation of a SLAM system for the robot using a particle filter, which can be categorized into mapping, acting, and sensing functionality. All modified files for this section can be found in ``/src/slam``.

### *Mapping*
In ``mapping.cpp``, a discretized occupancy grid map in conjunction with an inverse sensor model is used to map out the surroundings. Operates under the assumption that its information about its location is accurate.
- LiDAR scan is compensated for movement using odometry reading using
- Bresenham's line algorithm determines cells occupied by each ray
  - If beam passes through, it's likely empty, so occupancy log odds is decreased
  - If beam ends, it's likely occupied, so log odds are increased

### *Acting*
In ``action_model.cpp``, the **sample_motion_model_odometry** algorithm from *Probabilistic Robotics* is used to model the imprecision of odometry readings and allow the propagation of the particles through the environment.
- Noise model is characterized by four alpha coefficients, which were tuned consistently to optimize performance
- Algorithm uses rotation-translation rotation and applies noise at each step

The initialization of the particle filter occurs in ``particle_filter.cpp`` and uses its own set of noise constants to create the point cloud at the beginning.

### *Sensing*
In ``sensor_model.cpp``, a simplified version of the **beam_range_finder_model** algorithm from *Probabilistic Robotics* is used to evaluate the weight of each particle and update the robot's understanding of its position after the action model has been applied.
- Each particle start with a log odds value of 0, and every ray (approximately 100) in the current LiDAR scan is used to increment its value
- If the end of the beam...
  - **Coincides** or nearly coincides with an occupied cell: we decrease log odds by very minimally
  - Falls **before** an obstacle: we decrement odds by a larger amount, penalizing its incorrectness, though not severely due to possible noise errors or missed obstacles in current map construction
  - Falls **after** an obstacle (and before max beam distance): we decrement even more, penalizing severely, since this indicates the robot is unlikely to be located at that particle's location
- We convert from log odds into a decimal and return it as output

In ``particle_filter.cpp``, these values are calculated, normalized for all particles, and assigned as weights. It determines the robot's position with a weighted average, and also resamples according to those weights with a low-variance resampler algorithm.

## Part 2: Path Planning
