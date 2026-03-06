#!/bin/bash
# 2026.3.5

# sudo apt-get install wmctrl

gnome-terminal --tab --title="决策"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash; 
  cd ~//Decision/;
  source install/setup.bash; 
  ros2 launch dummy_data_reconfigure param_tuning.launch.py;
  ros2 run decision_process decision_process;exec bash"

gnome-terminal --tab --title="导航定位" -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash; 


gnome-terminal --tab --title="自瞄" -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash; 
  cd ~/SRVL_26/;
  ./bin/srvl_sentry; exec bash"
  