# Autonomous Racing Car Simulation

A ROS-based simulation project for developing and testing autonomous racing car algorithms in Gazebo. The system is structured into modular subsystems for mapping, localization, path planning, control, and a mode-selection plugin.

---

## Project Overview

This repository hosts all code, configurations, and data needed to simulate an autonomous racing car on a virtual track using ROS Noetic and Gazebo. The goal is to evaluate different driving modes and control strategies in a reproducible environment.

## Technologies & Algorithms per Subsystem

* **Localization**

  * **Description:** Estimates the vehicle's pose based on sensor inputs (e.g., odometry, IMU).
  * **Tech Stack:** ROS C++/Python node (`localization_node`), `tf2_ros`, `rospy` or `roscpp`.
  * **Key Algorithm:** Extended Kalman Filter (EKF) for sensor fusion.

* **Path Planning**

  * **Description:** Computes a sequence of waypoints for the vehicle to follow.
  * **Tech Stack:** ROS Python node (`pure_pursuit.py`), `rospy`.
  * **Key Algorithm:** RRT\* modified for the racing tracks.

* **Control**

  * **Description:** Executes low-level control commands (steering, throttle, braking) to follow the planned path.
  * **Tech Stack:** ROS C++ node (`pid_controller`), `ros_control` interface.
  * **Key Algorithm:** PID control loop regulating lateral and longitudinal errors.

* **Visualization**

  * **Description:** Provides realtime visualization of the map, planned path, vehicle pose, and sensor data.
  * **Tech Stack:** RViz configuration (`.rviz` file), `rosrun rviz rviz`.

* **Mode Selector (PLUGINS)**

  * **Description:** CLI-based selector to choose the driving mode before launch (e.g., Track Drive, Autonomy).
  * **Tech Stack:** Python script (`ami2.py`), `argparse` or simple text menu.
  * **Current Functionality:** Only **TRACK DRIVE** mode is implemented and tested.

---

## Setup & Run Instructions

Follow these steps to get the simulation up and running:

1. **Create a Catkin Workspace**

   ```bash
   mkdir -p ~/autonomous_racing_ws/src
   cd ~/autonomous_racing_ws/src
   ```

2. **Clone the Repository**

   ```bash
   git clone https://github.com/HazemBelal/Autonomous-RacingCar.git
   ```

3. **Build the Workspace**

   ```bash
   cd ~/autonomous_racing_ws
   catkin_make
   ```

4. **Source the Workspace**

   ```bash
   source devel/setup.bash
   ```

5. **Launch the Gazebo Simulation**
   To open the simulation with the *small track* configuration, run:

   ```bash
   roslaunch aamfsd_gazebo small_track.launch
   ```

6. **Select Driving Mode**
   In a new terminal (workspace sourced), launch the mode selector script:

   ```bash
   rosrun PLUGINS ami2.py
   ```

   * Use the interactive menu to choose **TRACK DRIVE** mode (the only fully supported mode).

7. **System Operation**
   Once the mode is selected, the full-stack nodes (mapping, localization, planning, control, visualization) will start under the chosen configuration.

---

## Notes & Troubleshooting

* Ensure ROS Noetic and Gazebo are installed and compatible with Ubuntu 20.04.
* Verify Python dependencies (if any) are installed: `pip3 install numpy rospy`
* If topics or nodes fail to start, check log output (`roslaunch` console) for missing dependencies or namespace mismatches.
* Customize CSV map files in `src/aamfsd_gazebo/maps/` for different track layouts.

---
