# ROS 2 Publisher and Subscriber for NU ID

This package (`id_publisher`) contains a ROS 2 C++ publisher and a subscriber node.
The publisher sends the digits of the NU ID "201769393" one by one on the topic named "Balapan".
The publisher initially sends at 1 Hz, and then later at 100 Hz. The current configuration is 100 Hz.
The subscriber listens to the "Balapan" topic and prints the received integer digits.

## Setup

First, ensure your ROS 2 Jazzy environment is sourced. If not, open a new terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
```

Then, source your workspace setup file. This makes your package executables available. From your `catkin_ws` directory, run:

```bash
source install/setup.bash
```

## Running the Nodes

To see the publisher and subscriber working simultaneously, you need to open two separate terminal windows.

### Terminal 1: Run the Publisher

In the first terminal, navigate to your `catkin_ws` directory and run the following commands:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run id_publisher id_publisher_node
```

The publisher node will start publishing the digits of the ID "201769393" at 100 Hz. You will see output similar to:

```
[INFO] [1678886400.000000000] [id_publisher_node]: Publishing: '2'
[INFO] [1678886400.010000000] [id_publisher_node]: Publishing: '0'
...
```

### Terminal 2: Run the Subscriber

In the second terminal, navigate to your `catkin_ws` directory and run the following commands:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run id_publisher subscriber
```

The subscriber node will start receiving and printing the digits published by the `id_publisher_node`. You will see output similar to:

```
[INFO] [1678886400.000000000] [rclcpp]: I heard: '2'
[INFO] [1678886400.010000000] [rclcpp]: I heard: '0'
...
```
