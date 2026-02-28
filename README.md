**Project Overview**

This project is a hands-on exploration of building a clean, modular, and reproducible data-processing pipeline using ROS 2 (Humble). Instead of focusing on a full navigation stack immediately, the goal here is to deeply understand how data flows through a robotic system — from sensing to processing to logging and analysis.

The repository demonstrates how to design ROS 2 nodes properly, configure them with parameters, orchestrate them using launch files, record experiments with rosbag2, and generate structured datasets for offline evaluation. It is intentionally simple in functionality but strong in architecture — reflecting how real robotics systems are built step by step.

**ROS 2 Modular Data Pipeline**

A modular ROS 2 (Humble) data-processing framework demonstrating:

- Publisher node (sensor simulation)

- Subscriber/processing node

- Parameterized processing (scale_factor)

- Reset service

- Launch-based orchestration

- rosbag2 recording & replay

- Automatic CSV experiment logging

- Post-processing and plotting pipeline

**Architecture**

sensor_publisher
        ↓
   /sensor_value
        ↓
processor_node
        ↓
 /processed_value
        ↓
 CSV Logger
        ↓
 Plotting Script

**Features**

- Modular ROS 2 node design

- Runtime parameter tuning

- Service-based state reset

- Reproducible experiment logging

- Offline data replay

- Statistical analysis & visualization

**Installation**

cd ~/ros2_ws/src

git clone https://github.com/YOUR_USERNAME/ros2-modular-data-pipeline.git

cd ~/ros2_ws

colcon build

source install/setup.bash

**Run**

Launch the file with the following command:

ros2 launch ros2_modular_data_pipeline pipeline.launch.py


**Manual Run:**

ros2 run ros2_modular_data_pipeline sensor_publisher

ros2 run ros2_modular_data_pipeline processor_node --ros-args -p scale_factor:=3.0

**Logging**

Logs are saved automatically to: ~/ros2_logs/

**Plotting**

Plot the CSV logged data using the following command:

python3 plot_csv.py <path_to_csv>
