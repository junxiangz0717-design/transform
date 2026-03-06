gnome-terminal --tab --title="决策"  -- bash -c "wmctrl -r :ACTIVE: -b toggle,above; 
  source /opt/ros/jazzy/setup.bash; 
  cd ~/transform/Decision/;
  source install/setup.bash; 
  ros2 launch dummy_data_reconfigure param_tuning.launch.py;
  ros2 run decision_process decision_process;exec bash"