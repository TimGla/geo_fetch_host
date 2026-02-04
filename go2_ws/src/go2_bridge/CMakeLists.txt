cmake_minimum_required(VERSION 3.8)
project(go2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(unitree_sdk2 REQUIRED)

add_executable(go2_bridge_node src/main.cpp)

target_include_directories(go2_bridge_node PUBLIC /opt/unitree_robotics/include)
target_link_libraries(go2_bridge_node unitree_sdk2)
ament_target_dependencies(go2_bridge_node rclcpp std_srvs unitree_sdk2)

install(TARGETS
  go2_bridge_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
