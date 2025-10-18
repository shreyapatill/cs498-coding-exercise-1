# CS498 Mobile Robotics - Coding Exercise 1

## Submission Files

This repository contains the completed coding exercise 1 for CS498 Mobile Robotics course.

### Files Included:

1. **coding_ex1_part1.py** - Wheel odometry implementation
2. **coding_ex1_part2.py** - GPS odometry implementation  
3. **utils.py** - Utility functions including quaternion conversion and GPS coordinate transformation
4. **results_part1.txt** - Wheel odometry trajectory data
5. **results_part2.txt** - GPS odometry trajectory data

### Implementation Details:

- **Part 1**: Uses wheel encoder data and gyroscope for odometry calculation
- **Part 2**: Uses GPS coordinates converted to local Cartesian coordinates for position
- Both implementations use gyroscope data for heading calculation
- Fixed timestep (dt = 0.1) used for integration
- Proper quaternion representation for robot orientation

### Results:

- **Part 1**: Robot trajectory shows significant displacement due to wheel odometry integration
- **Part 2**: Robot trajectory shows minimal displacement using absolute GPS positioning
- Both implementations generate complete trajectory data for the 231-second rosbag duration

## Usage

To run the nodes:

```bash
# Terminal 1: Play rosbag
ros2 bag play solar_house/solar_house.db3

# Terminal 2: Run wheel odometry
ros2 run mobile_robotics coding_ex1_part1

# Terminal 3: Run GPS odometry  
ros2 run mobile_robotics coding_ex1_part2
```

## Visualization

Use RViz2 to visualize the trajectories:
1. Set Fixed Frame to 'odom'
2. Add Odometry display for /odom topic
3. Add Path display for /odom topic with Buffer Length 10000
