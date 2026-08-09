# Sourced by every background stack process. Non-interactive ssh reads neither
# ~/.bashrc (ROS + gz paths) nor ~/.profile (venv + sim_vehicle.py PATH).
source /opt/ros/jazzy/setup.bash
source $HOME/huitzilin_ws/install/setup.bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}
export PATH=$HOME/ardupilot/Tools/autotest:$PATH
