cmake_minimum_required(VERSION 3.16)
project(robocup_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)


set(DEPS
    behaviortree_ros2
    btcpp_ros2_interfaces
    geometry_msgs)

include_directories(include)

# Executor
add_executable(bt src/bt.cpp)
ament_target_dependencies(bt ${DEPS})
install(TARGETS
  bt
  DESTINATION lib/${PROJECT_NAME}
)

# TwistAction
add_library(twist_action SHARED src/twist_action.cpp)
target_compile_definitions(twist_action PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(twist_action ${DEPS})
install(TARGETS
  twist_action
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS
  twist_action
  LIBRARY DESTINATION share/${PROJECT_NAME}/plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/plugins
)

# Resources
install(DIRECTORY launch config behaviors
    DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${DEPS})

ament_package()