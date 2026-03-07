gnome-terminal --tab --title="决策调参"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash; 
  cd ~/transform/msg_process/;
  source install/setup.bash
  cd ~/transform/Decision/;
  source install/setup.bash; 
  ros2 launch dummy_data_reconfigure param_tuning.launch.py;exec bash"

gnome-terminal --tab --title="决策开始"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash; 
  cd ~/transform/msg_process/;
  source install/setup.bash
  cd ~/transform/Decision/;
  source install/setup.bash; 
  ros2 run decision_process decision_process;exec bash"