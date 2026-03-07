#!/bin/bash
# 2026.3.5

# sudo apt-get install wmctrl

gonme-terminal --tab --title="决策开始"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash; 
  cd ~/transform/Decision/;
  source install/setup.bash; 
  ros2 run deision_process decision_process;exec bash"

gnome-terminal --tab --title="导航定位" -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash;" 


gnome-terminal --tab --title="自瞄" -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash;
  cd ~/SRVL_26/;
  source install_colcon/setup.bash;
  ./bin/srvl_sentry; exec bash"
  