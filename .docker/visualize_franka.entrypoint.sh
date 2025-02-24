#!/bin/bash
args=$*
shift $#
cd /workspaces
colcon build --packages-select franka_description franka_vis_tools
source install/setup.bash

# Debug information
echo "Checking package and executables..."
ros2 pkg list | grep franka_vis_tools
ros2 pkg executables franka_vis_tools

echo "Checking Python module location..."
python3 -c "import franka_vis_tools; print(franka_vis_tools.__file__)"

echo "Checking for executables in paths..."
find /workspaces/install -name "sphere_publisher" -type f
find /workspaces/install -name "custom_marker_publisher" -type f

# Try multiple methods to run the node
echo "Attempting to run the node..."
(ros2 run franka_vis_tools sphere_publisher || \
 /workspaces/install/franka_vis_tools/lib/franka_vis_tools/sphere_publisher || \
 python3 -m franka_vis_tools.sphere_publisher) &
# ros2 run franka_vis_tools sphere_publisher &
# ros2 run franka_vis_tools custom_marker_publisher &
(ros2 run franka_vis_tools custom_marker_publisher || \
 /workspaces/install/franka_vis_tools/lib/franka_vis_tools/custom_marker_publisher || \
 python3 -m franka_vis_tools.custom_marker_publisher) &

# Continue with launch
ros2 launch franka_description visualize_franka.launch.py ${args}