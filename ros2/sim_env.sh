# Source this in every terminal used for the simulation:
#   source SeniorDesign_4a/ros2/sim_env.sh
#
# Sets up ROS 2 Jazzy plus this workspace, and keeps ROS traffic on this
# machine so the simulation nodes find each other without a discovery server.

_ros2_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# build/, install/ and log/ live in the repo root (SeniorDesign_4a).
_colcon_ws="$(cd "$_ros2_dir/.." && pwd)"

source /opt/ros/jazzy/setup.bash
if [ -f "$_colcon_ws/install/setup.bash" ]; then
  source "$_colcon_ws/install/setup.bash"
else
  echo "sim_env.sh: $_colcon_ws/install not found. Run 'colcon build --symlink-install' in $_colcon_ws first." >&2
fi

# A ROS_DISCOVERY_SERVER set in the shell profile makes nodes invisible to each
# other unless that server is running and reachable.
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

export WEBOTS_HOME=/usr/local/webots

unset _ros2_dir _colcon_ws
