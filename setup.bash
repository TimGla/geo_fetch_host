#!/bin/bash

# Define the base path to your workspace
BASE_PATH="$HOME/geo_fetch_host"

echo "Launching ROS2 environment in separate terminals..."

# Terminal 1: Micro-ROS Agent
gnome-terminal --tab --title="Micro-ROS Agent" -- bash -c "
    cd $BASE_PATH/microros_ws/install && \
    source local_setup.bash && \
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200; 
    exec bash"

# Terminal 2: Rosbridge Server
gnome-terminal --tab --title="Rosbridge" -- bash -c "
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    call_services_in_new_thread:=True \
    default_call_service_timeout:=5.0; \
    exec bash"

# Terminal 3: Web Dashboard Server
gnome-terminal --tab --title="HTTP Server" -- bash -c "
    cd $BASE_PATH/web_dashboard && \
    python3 -m http.server 7000; \
    exec bash"

# Terminal 4: Go2 Bringup
gnome-terminal --tab --title="Go2 Bringup" -- bash -c "
    ros2 launch go2_bringup go2.launch.py; \
    exec bash"

echo "All processes started."