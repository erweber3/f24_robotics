Terminal 1 Commands: 
source /opt/ros/humble/setup.bash

if using windows:
export WEBOTS_HOME=/mnt/c/Program\ Files/Webots
if using mac: 
export WEBOTS_HOME=/Applications/Webots.app
python3 local_simulation_server.py

cd f24_robotics
colcon build
source install/setup.bash

ros2 launch webots_ros2_homework1_python f23_robotics_1_launch.py

Terminal 2 Commands:
source /opt/ros/humble/setup.bash
cd f24_robotics
ros2 run webots_ros2_homework1_python webots_ros2_homework1_python

Other Notes:
Ensure that you kill all processes after they are complete otherwise this will interfere with your next run


