# SITL-only env: hz_env.sh plus the ardupilot venv (mavproxy.py lives there).
# Kept separate so the venv never shadows ROS python for the T3 stack.
source "$(dirname "${BASH_SOURCE[0]}")/hz_env.sh"
source $HOME/venv-ardupilot/bin/activate
