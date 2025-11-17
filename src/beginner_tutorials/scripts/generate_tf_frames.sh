#!/bin/bash
# Script to generate TF frames PDF and save to results directory

# Get the workspace root (assuming script is in package/scripts/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
RESULTS_DIR="$WORKSPACE_ROOT/results"

# Create results directory if it doesn't exist
mkdir -p "$RESULTS_DIR"

# Change to workspace root
cd "$WORKSPACE_ROOT"

# Source ROS 2 environment
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then
    source install/setup.bash
fi

# Start talker in background
echo "Starting talker node..."
ros2 run beginner_tutorials talker > /dev/null 2>&1 &
TALKER_PID=$!

# Wait for TF frames to be published
sleep 3

# Generate the frames PDF
echo "Generating TF frames PDF..."
ros2 run tf2_tools view_frames

# Move to results directory if frames.pdf was created
if [ -f frames.pdf ]; then
    mv frames.pdf "$RESULTS_DIR/tf_frames.pdf"
    echo "TF frames PDF saved to $RESULTS_DIR/tf_frames.pdf"
    
    # Clean up any .gv files
    rm -f frames_*.gv
else
    echo "Error: frames.pdf not generated"
    kill $TALKER_PID 2>/dev/null
    exit 1
fi

# Stop the talker
kill $TALKER_PID 2>/dev/null
wait $TALKER_PID 2>/dev/null

echo "Done!"

