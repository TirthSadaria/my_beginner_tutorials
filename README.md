# Beginner Tutorials

A ROS 2 package demonstrating basic publisher and subscriber nodes with services, logging, and launch files for ENPM700.

## Overview

This package contains two ROS 2 nodes:
- **talker**: A publisher node that publishes string messages to the `topic` topic. It includes a service server (`change_string`) that allows changing the base output string dynamically at runtime.
- **listener**: A subscriber node that subscribes to the `topic` topic and prints received messages with various logging levels.

The nodes demonstrate:
- Basic ROS 2 communication patterns using member function callbacks
- Service server implementation for dynamic configuration
- Comprehensive logging at all five levels (DEBUG, INFO, WARN, ERROR, FATAL)
- Parameter-based configuration
- Launch file usage with command-line arguments

## Dependencies and Assumptions

### Required Dependencies
- **ROS 2 Humble Hawksbill** (or compatible ROS 2 distribution)
- **rclcpp**: ROS 2 C++ client library
- **std_msgs**: Standard ROS 2 message types
- **ament_cmake**: ROS 2 build system
- **rosidl_default_generators**: For generating service interfaces
- **rosidl_default_runtime**: Runtime support for generated interfaces
- **launch_ros**: For Python launch files

### System Requirements
- Linux operating system (tested on Ubuntu 22.04)
- C++17 compatible compiler (GCC 7+ or Clang 5+)
- CMake 3.8 or higher

## Building the Package

### Cloning into an Existing Colcon Workspace

If you're cloning this package into an existing colcon workspace:

1. **Navigate to your colcon workspace `src` directory**:
   ```bash
   cd /path/to/your/colcon_workspace/src
   ```

2. **Clone the repository**:
   ```bash
   git clone <repository_url> beginner_tutorials
   ```

3. **Navigate to workspace root**:
   ```bash
   cd /path/to/your/colcon_workspace
   ```

4. **Source ROS 2 environment** (if not already sourced):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

5. **Install dependencies** (if needed):
   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

6. **Build the package**:
   ```bash
   colcon build --packages-select beginner_tutorials
   ```

7. **Source the workspace**:
   ```bash
   source install/setup.bash
   ```

### Building in Current Workspace

If you're already in the workspace containing this package:

1. **Source ROS 2 environment** (if not already sourced):
   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. **Navigate to workspace root**:
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

### Option 1: Using Launch File (Recommended)

The launch file starts both nodes simultaneously and accepts a frequency parameter:

```bash
ros2 launch beginner_tutorials talker_listener.launch.py frequency:=500
```

**Parameters:**
- `frequency`: Publishing frequency in milliseconds (default: 500ms)
  - Example: `frequency:=1000` for 1 second intervals
  - Example: `frequency:=250` for 250ms intervals

### Option 2: Running Nodes Individually

#### Terminal 1: Run the Publisher (talker)
```bash
ros2 run beginner_tutorials talker
```

This will start publishing messages to the `topic` topic. You should see output like:
```
[INFO] [minimal_publisher]: Publishing: 'ENPM700 Testing Publisher Output  0'
[INFO] [minimal_publisher]: Publishing: 'ENPM700 Testing Publisher Output  1'
...
```

#### Terminal 2: Run the Subscriber (listener)
```bash
ros2 run beginner_tutorials listener
```

This will subscribe to the `topic` topic and print received messages:
```
[INFO] [minimal_subscriber]: I heard: 'ENPM700 Testing Publisher Output  0'
[INFO] [minimal_subscriber]: I heard: 'ENPM700 Testing Publisher Output  1'
...
```

## Using the Service

The talker node provides a service called `change_string` that allows you to change the base output string dynamically.

### List Available Services
```bash
ros2 service list
```

You should see `/change_string` in the list.

### Check Service Type
```bash
ros2 service type /change_string
```

Expected output: `beginner_tutorials/srv/ChangeString`

### View Service Interface
```bash
ros2 interface show beginner_tutorials/srv/ChangeString
```

Expected output:
```
string new_string
---
bool success
```

### Call the Service

**Using ros2 service call command:**
```bash
ros2 service call /change_string beginner_tutorials/srv/ChangeString "{new_string: 'Hello World'}"
```

**Example with different strings:**
```bash
# Change to a custom message
ros2 service call /change_string beginner_tutorials/srv/ChangeString "{new_string: 'Custom Message'}"
```

**Verify the change:**
After calling the service, the talker node will immediately start publishing messages with the new base string. You should see output like:
```
[INFO] [minimal_publisher]: Base string changed from 'ENPM700 Testing Publisher Output  ' to 'Custom Message'
[INFO] [minimal_publisher]: Publishing: 'Custom Message0'
[INFO] [minimal_publisher]: Publishing: 'Custom Message1'
...
```

### Service Response
The service returns:
- `success: true` if the string was changed successfully
- `success: false` if an empty string was provided

## Logging Levels

Both nodes use all five ROS 2 logging levels with the `_STREAM` API:

- **DEBUG**: Detailed diagnostic information (e.g., initialization details, message preparation)
- **INFO**: General informational messages (e.g., normal operation status)
- **WARN**: Warning messages (e.g., empty strings, long messages)
- **ERROR**: Error conditions (e.g., invalid parameters, error keywords in messages)
- **FATAL**: Fatal/critical errors (e.g., system instability conditions)

### Viewing Logs with rqt_console

1. **Start the nodes** (using launch file or individually)

2. **Open rqt_console**:
   ```bash
   ros2 run rqt_console rqt_console
   ```

3. **Filter by logger level**:
   - Use the severity filter in the GUI (Exclude messages section)
   - Select/deselect different levels (DEBUG, INFO, WARN, ERROR, FATAL) to filter messages
   - Screenshots should be captured showing at least two different logger level configurations

### Setting Logger Level for Nodes

Both nodes support a `logger_level` parameter that can be set at runtime:

**Set logger level to DEBUG:**
```bash
ros2 param set /minimal_publisher logger_level DEBUG
ros2 param set /minimal_subscriber logger_level DEBUG
```

**Set logger level to WARN:**
```bash
ros2 param set /minimal_publisher logger_level WARN
ros2 param set /minimal_subscriber logger_level WARN
```

**Valid logger level values:**
- `DEBUG` - Shows all messages including detailed diagnostics
- `INFO` - Shows informational messages and above
- `WARN` - Shows warnings and above
- `ERROR` - Shows errors and above
- `FATAL` - Shows only fatal messages

**Check current logger level:**
```bash
ros2 param get /minimal_publisher logger_level
ros2 param get /minimal_subscriber logger_level
```

**Note:** The logger level can be changed at runtime without restarting the nodes. The change takes effect immediately and you'll see a confirmation message in the node output.

## Package Structure

```
beginner_tutorials/
├── CMakeLists.txt          # Build configuration
├── package.xml             # Package manifest
├── LICENSE                 # Apache 2.0 License
├── src/
│   ├── publisher_member_function.cpp  # Talker node with service
│   └── subscriber_member_function.cpp # Listener node
├── srv/
│   └── ChangeString.srv    # Service definition
└── launch/
    └── talker_listener.launch.py  # Launch file
```

## Code Style

This project follows the **Google C++ Style Guide** with the following conventions:
- Class names in PascalCase (`MinimalPublisher`, `MinimalSubscriber`)
- Function names in PascalCase (`TimerCallback`, `TopicCallback`)
- Member variables end with underscore (`count_`, `publisher_`, `base_string_`)
- 2-space indentation
- Opening braces on the same line
- Explicit constructors
- Const correctness where applicable

**Note**: When ROS API conventions conflict with Google style guide, ROS API conventions are followed.

## Documentation

All source files include Doxygen-compatible comments:
- File-level documentation with `@file` tags
- Class documentation with `@brief` descriptions
- Function documentation with parameter and return descriptions
- Member variable documentation with inline comments

## License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

