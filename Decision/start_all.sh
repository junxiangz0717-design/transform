#!/bin/bash

# 1. Start Gazebo Simulation (Open the Window)
echo "Step 1: Starting Gazebo Simulation..."
terminator -T "1. Gazebo Simulation" -e "bash -c 'source /opt/ros/humble/setup.bash; source /home/yu/26-sentry/rm_sim_ws/install/setup.bash; ros2 launch /home/yu/26-sentry/rm_sim_ws/src/bot_description/launch/gazebo_sim_2026L.launch.py; exec bash'" &

# Wait for the first window to be ready
sleep 2

# 2. Start Navigation2 (Add Tab)
echo "Step 2: Starting Navigation2 (Including RViz)..."
terminator --new-tab -T "2. Navigation2" -e "bash -c 'sleep 3; source /opt/ros/humble/setup.bash; source /home/yu/26-sentry/rm_sim_ws/install/setup.bash; ros2 launch /home/yu/26-sentry/rm_sim_ws/src/bot_navigation2/launch/navigation2_26L.launch.py; exec bash'" &

# 3. Start Param Tuning (Add Tab)
echo "Step 3: Starting Parameter Tuning (Including Rqt)..."
terminator --new-tab -T "3. Param Tuning" -e "bash -c 'sleep 8; source /opt/ros/humble/setup.bash; source /home/yu/26-sentry/Decision/install/setup.bash; ros2 launch /home/yu/26-sentry/Decision/dummy_data_reconfigure/launch/param_tuning.launch.py; exec bash'" &

# 4. Start Decision Process (Add Tab)
echo "Step 4: Starting Decision Process..."
terminator --new-tab -T "4. Decision Process" -e "bash -c 'sleep 10; source /opt/ros/humble/setup.bash; source /home/yu/26-sentry/msg_process/install/setup.bash; source /home/yu/26-sentry/Decision/install/setup.bash; ros2 run decision_process decision_process; exec bash'" &

echo "==========================================================="
echo "All processes initiated in ONE Terminator window."
echo "Use ./all_kill.sh to stop all processes."
echo "==========================================================="
