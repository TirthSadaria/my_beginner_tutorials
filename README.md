# Beginner Tutorials

A ROS 2 package demonstrating basic publisher and subscriber nodes with services, logging, TF2 transforms, integration testing, and bag file recording for ENPM700.

## Overview

This package contains two ROS 2 nodes:
- **talker**: A publisher node that publishes string messages to the `topic` topic. It includes a service server (`change_string`) that allows changing the base output string dynamically at runtime, and broadcasts TF2 transforms for the `/talk` frame relative to `/world`.
- **listener**: A subscriber node that subscribes to the `topic` topic and prints received messages with various logging levels.

The package demonstrates:
- Basic ROS 2 communication patterns using member function callbacks
- Service server implementation for dynamic configuration
- Comprehensive logging at all five levels (DEBUG, INFO, WARN, ERROR, FATAL)
- Parameter-based configuration
- Launch file usage with command-line arguments
- **TF2 transform broadcasting** with time-variant transforms
- **Catch2 integration testing** (Level 2 tests)
- **ROS 2 bag file recording and playback** with launch file support

## Dependencies and Assumptions

### Required Dependencies
- **ROS 2 Humble Hawksbill** (or compatible ROS 2 distribution)
- **rclcpp**: ROS 2 C++ client library
- **std_msgs**: Standard ROS 2 message types
- **geometry_msgs**: Geometry message types for TF2
- **tf2**: TF2 library
- **tf2_ros**: TF2 ROS integration
- **tf2_geometry_msgs**: TF2 geometry message conversions
- **ament_cmake**: ROS 2 build system
- **rosidl_default_generators**: For generating service interfaces
- **rosidl_default_runtime**: Runtime support for generated interfaces
- **launch_ros**: For Python launch files
- **Catch2**: C++ testing framework (via ament_catch2)

### System Requirements
- Linux operating system (tested on Ubuntu 22.04)
- C++17 compatible compiler (GCC 7+ or Clang 5+)
- CMake 3.8 or higher
- Graphviz (for `view_frames` tool)

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
   cd /path/to/your/colcon_workspace
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

This will start publishing messages to the `topic` topic and broadcasting TF2 transforms. You should see output like:
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

## Inspecting TF2 Frames

The talker node broadcasts a TF2 transform for the `/talk` frame with parent `/world`. The transform includes time-variant translation and rotation.

### Viewing TF Transforms with tf2_echo

1. **Start the talker node**:
   ```bash
   ros2 run beginner_tutorials talker
   ```

2. **In another terminal, echo the transform**:
   ```bash
   ros2 run tf2_ros tf2_echo world talk
   ```

   You should see output showing the transform between `world` and `talk` frames:
   ```
   At time 1234567890.123
   - Translation: [2.000, 1.000, 0.000]
   - Rotation: in Quaternion [0.000, 0.000, 0.707, 0.707]
   ```

3. **Press Ctrl+C** to stop viewing transforms.

### Generating TF Frames PDF

There are multiple ways to generate the TF frames PDF:

#### Option 1: Using Launch File (Recommended)

The easiest way is to use the launch file with the `generate_tf_frames` argument:

```bash
ros2 launch beginner_tutorials bag_record.launch.py generate_tf_frames:=true
```

This will:
- Start both talker and listener nodes
- Wait 5 seconds for TF frames to accumulate
- Automatically run `view_frames` to generate the PDF
- Save the PDF to `results/tf_frames.pdf`
- Clean up temporary files

The launch file will continue running after generation (press Ctrl+C to stop).

#### Option 2: Using the Script

You can also use the provided script:

```bash
ros2 run beginner_tutorials generate_tf_frames.sh
```

This script:
- Starts the talker node automatically
- Waits for TF frames to be published
- Generates the PDF using `view_frames`
- Saves it to `results/tf_frames.pdf`
- Stops the talker node automatically

#### Option 3: Manual Method

1. **Start the talker node** (in one terminal):
   ```bash
   ros2 run beginner_tutorials talker
   ```

2. **In another terminal, generate the frames PDF**:
   ```bash
   ros2 run tf2_tools view_frames
   ```

   This will:
   - Listen to TF data for 5 seconds
   - Generate a PDF file named `frames.pdf` in the current directory
   - Show the TF tree structure with `/world` as parent and `/talk` as child

3. **Move PDF to results directory**:
   ```bash
   mkdir -p results
   mv frames.pdf results/tf_frames.pdf
   rm -f frames_*.gv  # Clean up temporary files
   ```

4. **View the generated PDF**:
   ```bash
   evince results/tf_frames.pdf
   ```
   Or use any PDF viewer of your choice.

5. **Stop the talker node** (Ctrl+C in the first terminal).

**Note**: The generated PDF shows the TF tree structure with `/world` as parent and `/talk` as child frame, demonstrating the time-variant transform relationship.

## Running ROS 2 Tests

This package includes Catch2 integration tests (Level 2 tests) that verify the talker node functionality.

### Running All Tests

```bash
colcon test --packages-select beginner_tutorials
```

### Viewing Test Results

After running tests, view the results:

```bash
colcon test-result --verbose
```

### Running Tests with Detailed Output

```bash
colcon test --packages-select beginner_tutorials --event-handlers console_direct+
```

### Expected Test Output

The integration test verifies that:
- The talker node can be instantiated
- The node publishes messages to the `/topic` topic
- At least one message is received within a reasonable timeout

You should see output indicating that the test passed:
```
1 test case - 1 passed
```

## Recording and Playing Back Bag Files

### Recording Bag Files with Launch File

The package includes a launch file that can record all topics to a ROS 2 bag file and optionally generate TF frames PDF.

#### Enable Bag Recording

```bash
ros2 launch beginner_tutorials bag_record.launch.py record:=true frequency:=500
```

This will:
- Start both talker and listener nodes
- Begin recording all topics (`-a` flag) to a bag file
- Create a bag directory named `rosbag2_YYYY_MM_DD-HH_MM_SS` in the current directory

#### Disable Bag Recording

```bash
ros2 launch beginner_tutorials bag_record.launch.py record:=false frequency:=500
```

This will:
- Start both talker and listener nodes
- **Not** record any topics

#### Generate TF Frames PDF with Launch File

You can also generate the TF frames PDF directly from the launch file:

```bash
ros2 launch beginner_tutorials bag_record.launch.py generate_tf_frames:=true
```

This will:
- Start both talker and listener nodes
- Wait 5 seconds for TF frames to accumulate
- Automatically run `view_frames` to generate the PDF
- Save the PDF to `results/tf_frames.pdf`
- Clean up temporary files

**Parameters:**
- `record`: Enable/disable bag recording (`true` or `false`, default: `false`)
- `generate_tf_frames`: Enable/disable TF frames PDF generation (`true` or `false`, default: `false`)
- `frequency`: Publishing frequency in milliseconds (default: 500ms)

**Example - Record bag and generate TF frames:**
```bash
ros2 launch beginner_tutorials bag_record.launch.py record:=true generate_tf_frames:=true
```

**Example - Record for 15 seconds:**
```bash
# Start recording
ros2 launch beginner_tutorials bag_record.launch.py record:=true

# Let it run for ~15 seconds, then press Ctrl+C to stop
```

### Inspecting Bag Files

After recording a bag file, inspect its contents:

1. **Find the bag directory**:
   ```bash
   ls -lt rosbag2_* | head -1
   ```

2. **Get bag information**:
   ```bash
   ros2 bag info rosbag2_YYYY_MM_DD-HH_MM_SS
   ```

   This will show:
   - Bag size
   - Duration
   - Number of messages
   - Topics recorded (should include `/topic`, `/tf`, `/rosout`, etc.)
   - Message counts per topic

3. **Save bag info to file** (optional):
   ```bash
   ros2 bag info rosbag2_YYYY_MM_DD-HH_MM_SS > bag_info.txt
   ```

### Playing Back Bag Files

To verify that the bag file contains valid messages, play it back with only the listener node:

1. **Terminal 1 - Start ONLY the listener node**:
   ```bash
   ros2 run beginner_tutorials listener
   ```

2. **Terminal 2 - Play the bag file**:
   ```bash
   ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS
   ```

3. **Verify output**: You should see the listener printing messages from the bag file:
   ```
   [INFO] [minimal_subscriber]: I heard: 'ENPM700 Testing Publisher Output  0'
   [INFO] [minimal_subscriber]: I heard: 'ENPM700 Testing Publisher Output  1'
   ...
   ```

4. **Stop both terminals** (Ctrl+C) when done.

**Note**: The talker node should **NOT** be running during playback. Only the listener should be active to verify that messages are being replayed from the bag file.

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
├── CMakeLists.txt                    # Build configuration
├── package.xml                        # Package manifest
├── LICENSE                            # Apache 2.0 License
├── include/
│   └── beginner_tutorials/
│       └── minimal_publisher.hpp     # Publisher class header
├── src/
│   ├── minimal_publisher.cpp         # Publisher class implementation
│   ├── publisher_member_function.cpp # Talker node entry point
│   └── subscriber_member_function.cpp # Listener node
├── srv/
│   └── ChangeString.srv               # Service definition
├── launch/
│   ├── talker_listener.launch.py      # Launch file for nodes
│   ├── bag_record.launch.py           # Launch file with bag recording and TF frames
│   └── generate_tf_frames.launch.py   # Launch file for TF frames generation only
├── scripts/
│   └── generate_tf_frames.sh          # Script to generate TF frames PDF
└── test/
    ├── talker_integration_test.cpp    # Catch2 integration test
    └── ament_cmake_test_stub/          # Test infrastructure
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
- C++11 features used where appropriate (auto keyword, range-based for loops, etc.)

**Note**: When ROS API conventions conflict with Google style guide, ROS API conventions are followed.

## Documentation

All source files include Doxygen-compatible comments:
- File-level documentation with `@file` tags
- Class documentation with `@brief` descriptions
- Function documentation with parameter and return descriptions
- Member variable documentation with inline comments
- License information in file headers

## License

This project is licensed under the **Apache License 2.0**. See the [LICENSE](LICENSE) file for details.

## Results Directory

The `results/` directory contains assignment artifacts:
- `tf_frames.pdf`: TF frames visualization generated using `view_frames`
- `bag_info.txt`: Output from `ros2 bag info` command
- `rosbag2_YYYY_MM_DD-HH_MM_SS/`: Recorded bag file directory

## Troubleshooting

### Library Not Found Error

If you encounter `error while loading shared libraries: libminimal_publisher_lib.so`:
- This should not occur as RPATH is set in CMakeLists.txt
- If it does occur, rebuild the package: `colcon build --packages-select beginner_tutorials`
- Ensure you've sourced the workspace: `source install/setup.bash`
- The launch file handles library paths automatically

### TF Frames Not Showing

Ensure the talker node is running and has been active for at least a few seconds before running `view_frames` or `tf2_echo`.

### Bag File Not Recording

Verify that `record:=true` is set in the launch file command. Check that the bag directory is being created in the current working directory.

### Tests Not Running

Ensure Catch2 is installed:
```bash
sudo apt install ros-humble-ament-catch2
```

Then rebuild the package:
```bash
colcon build --packages-select beginner_tutorials
```

## Testing Guide

### Quick Test Checklist

After building the package, verify all functionality:

1. **Test TF2 Broadcasting:**
   ```bash
   # Terminal 1
   ros2 run beginner_tutorials talker
   
   # Terminal 2 (wait a few seconds)
   ros2 run tf2_ros tf2_echo world talk
   ```
   Should show continuous transform data with non-zero translation and rotation.

2. **Test Integration Test:**
   ```bash
   colcon test --packages-select beginner_tutorials
   colcon test-result --verbose | grep talker_integration_test
   ```
   Should show test passed.

3. **Test Bag Recording:**
   ```bash
   ros2 launch beginner_tutorials bag_record.launch.py record:=true
   # Wait 15 seconds, then Ctrl+C
   ros2 bag info rosbag2_<timestamp> | grep "/topic"
   ```
   Should show `/topic` with message count > 0.

4. **Test Bag Playback:**
   ```bash
   # Terminal 1
   ros2 run beginner_tutorials listener
   
   # Terminal 2
   ros2 bag play rosbag2_<timestamp>
   ```
   Listener should print messages from bag.

5. **Test Service:**
   ```bash
   # Terminal 1
   ros2 run beginner_tutorials talker
   
   # Terminal 2
   ros2 service call /change_string beginner_tutorials/srv/ChangeString "{new_string: 'Test'}"
   ```
   Talker should immediately start using new string.

6. **Test TF Frames PDF Generation:**
   ```bash
   ros2 launch beginner_tutorials bag_record.launch.py generate_tf_frames:=true
   # Wait ~10 seconds for PDF generation
   ls -lh results/tf_frames.pdf
   ```
   PDF should exist in results directory.

### Repository Structure

The repository contains ONLY the `beginner_tutorials` ROS 2 package:
- No build artifacts (`build/`, `install/`, `log/` are in `.gitignore`)
- Source code in `src/beginner_tutorials/`
- Results/artifacts in `results/` directory
- All required files for building and running the package
