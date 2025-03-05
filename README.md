# Trajectory Visualization and Recording in ROS2

This repository provides a ROS2 package for recording and visualizing a robot's trajectory in Gazebo and RViz. The package includes nodes to read, save, and publish trajectory data in a CSV file while interacting with a ROS2 service.

## Installation

### Clone the repository into your ROS2 workspace:
```bash
cd ~/ros2_ws/src
git clone <repository_link>
cd ~/ros2_ws
colcon build --packages-select traj_visual
source install/setup.bash
```

### Ensure dependencies are installed:
```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Usage

### 1. Launch Gazebo and Spawn the Robot
```bash
ros2 launch bbot_description gazebo.launch.py
```
This launches Gazebo and spawns the robot model into the simulation.

### 2. Read and Visualize a Trajectory from a CSV File
```bash
ros2 run traj_visual traj_read --ros-args -p filename:="/path/to/your/file.csv"
```
For default file name:
```bash
ros2 run traj_visual traj_read
```
This node reads a trajectory stored in a CSV file and publishes the path for visualization in RViz.

### 3. Save and Publish the Robot’s Path in Real-Time
```bash
ros2 run traj_visual traj_saver
```
This node records the robot’s trajectory and simultaneously publishes it for visualization.

### 4. Request a Service to Start Recording a Trajectory
```bash
ros2 service call /traj_service custom_service/srv/TrajSw "{filename: 'trajectory.csv', duration: 5.0}"
```
This sends a request to the traj_service to record the robot’s trajectory into `trajectory.csv` for a duration of 5 seconds.

## Topics and Services

### Published Topics:
- `/trajectory_markers` → MarkerArray for visualizing the path in RViz.
- `/odom` → Used for extracting robot position data.

### Services:
- `/traj_service` → Allows dynamic recording of trajectories by requesting a filename and duration.

## Visualization in RViz

To visualize the trajectory:

1. Open RViz:
   ```bash
   ros2 run rviz2 rviz2
   ```
2. Add a **MarkerArray** display.
3. Set the topic to **/trajectory_markers**.

## License
This project is licensed under the MIT License.

