cd ~/shelfino_ws
colcon build --packages-select dubins_planner
source install/setup.bash
cd ~/shelfino_ws/src/Robot_Planning_Project 
ros2 run dubins_planner rec
python3 plot.py
