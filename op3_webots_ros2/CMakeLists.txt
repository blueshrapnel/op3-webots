cmake_minimum_required(VERSION 3.8)
project(op3_webots_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(service_msgs REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(webots_ros2_driver REQUIRED)

find_package(robotis_controller REQUIRED)
find_package(open_cr_module REQUIRED)
find_package(robotis_math REQUIRED)
find_package(op3_base_module REQUIRED)
find_package(op3_head_control_module REQUIRED)
find_package(op3_action_module REQUIRED)
find_package(op3_walking_module REQUIRED)
find_package(op3_direct_control_module REQUIRED)
find_package(op3_online_walking_module REQUIRED)
find_package(op3_tuning_module REQUIRED)

find_package(PkgConfig REQUIRED)

pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)

find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)


add_executable(op3_extern_controller 
  src/op3_extern_controller.cpp 
  src/extern_controller_main.cpp
)


ament_target_dependencies(op3_extern_controller
  ament_index_cpp
  rclcpp
  webots_ros2_driver
)

target_link_libraries(op3_extern_controller ${YAML_CPP_LIBRARIES})

add_executable(op3_manager src/op3_manager.cpp)
target_include_directories(op3_manager PUBLIC ${robotis_math_INCLUDE_DIRS})


ament_target_dependencies(op3_manager
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  rcl_interfaces
  service_msgs
  ament_index_cpp
  robotis_controller
  open_cr_module
  robotis_math
  op3_base_module
  op3_head_control_module
  op3_action_module
  op3_walking_module
  op3_direct_control_module
  op3_online_walking_module
  op3_tuning_module
)

target_link_libraries(op3_manager ${YAML_CPP_LIBRARIES})

install(TARGETS
  op3_extern_controller
  op3_manager
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  resource
  images
  protos
  worlds
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
