# Beginner Tutorials

A ROS 2 package demonstrating basic publisher and subscriber nodes for ENPM700.

## Overview

This package contains two ROS 2 nodes:
- **talker**: A publisher node that publishes string messages to the `topic` topic
- **listener**: A subscriber node that subscribes to the `topic` topic and prints received messages

The nodes demonstrate basic ROS 2 communication patterns using member function callbacks.

## Dependencies and Assumptions

### Required Dependencies
- **ROS 2 Humble Hawksbill** (or compatible ROS 2 distribution)
- **rclcpp**: ROS 2 C++ client library
- **std_msgs**: Standard ROS 2 message types
- **ament_cmake**: ROS 2 build system

### System Requirements
- Linux operating system (tested on Ubuntu 22.04)
- C++17 compatible compiler (GCC 7+ or Clang 5+)
- CMake 3.8 or higher

## Building the Package

1. **Source ROS 2 environment** (if not already sourced):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Navigate to your workspace root** (parent directory of `beginner_tutorials`):
   ```bash
   cd /home/tirth/ENPM700/ros2/my_beginner_tutorials
   ```

3. **Build the package**:
   ```bash
   colcon build --packages-select beginner_tutorials
   ```

4. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

## Running the Nodes

### Terminal 1: Run the Publisher (talker)
```bash
ros2 run beginner_tutorials talker
```

This will start publishing messages to the `topic` topic. You should see output like:
```
[INFO] [minimal_publisher]: Publishing: 'ENPM700 Testing Publisher Output  0'
[INFO] [minimal_publisher]: Publishing: 'ENPM700 Testing Publisher Output  1'
...
```

### Terminal 2: Run the Subscriber (listener)
```bash
ros2 run beginner_tutorials listener
```

This will subscribe to the `topic` topic and print received messages:
```
[INFO] [minimal_subscriber]: I heard: 'ENPM700 Testing Publisher Output  0'
[INFO] [minimal_subscriber]: I heard: 'ENPM700 Testing Publisher Output  1'
...
```

## Code Style

This project follows the **Google C++ Style Guide** with the following conventions:
- Class names in PascalCase
- Function names in PascalCase
- Member variables end with underscore
- 2-space indentation
- Opening braces on the same line
- Explicit constructors
- Const correctness where applicable

## License

Apache License 2.0

