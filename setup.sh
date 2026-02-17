#!/bin/bash

# Path to your workspace
BASE_PATH="$HOME/geo_fetch_host"
SESSION="robot_launch"

# Kill any existing session with the same name
tmux kill-session -t $SESSION 2>/dev/null

# Create a new session and start Terminal 1 (Micro-ROS)
tmux new-session -d -s $SESSION -n "ROS_STUFF" "bash -c 'cd $BASE_PATH/microros_ws/install && source local_setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200; exec bash'"

# Split the window for Terminal 2 (Rosbridge)
tmux split-window -h -t $SESSION "bash -c 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml call_services_in_new_thread:=True default_call_service_timeout:=5.0; exec bash'"

# Split the bottom left for Terminal 3 (HTTP Server)
tmux split-window -v -t $SESSION.0 "bash -c 'cd $BASE_PATH/web_dashboard && python3 -m http.server 7000; exec bash'"

# Split the bottom right for Terminal 4 (Go2 Bringup)
tmux split-window -v -t $SESSION.1 "bash -c 'ros2 launch go2_bringup go2.launch.py; exec bash'"

# Attach to the session so you can see it
tmux attach-session -t $SESSION