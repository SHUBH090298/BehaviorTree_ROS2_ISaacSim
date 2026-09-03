cmake_minimum_required(VERSION 3.8)
project(bt_thesis)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(DEPS rclcpp behaviortree_cpp sensor_msgs std_msgs geometry_msgs)

# main executor
add_executable(bt_executor src/bt_executor.cpp)
ament_target_dependencies(bt_executor ${DEPS})

# tool that writes the Groot2 node-model XML
add_executable(generate_model src/generate_model.cpp)
ament_target_dependencies(generate_model ${DEPS})

add_custom_command(TARGET generate_model POST_BUILD
  COMMAND generate_model ${CMAKE_CURRENT_SOURCE_DIR}/trees/bt_thesis_models.xml
  COMMENT "Regenerating Groot2 node model")

install(TARGETS bt_executor generate_model DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY trees config launch DESTINATION share/${PROJECT_NAME})

ament_package()
