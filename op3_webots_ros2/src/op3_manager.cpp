/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman */

/* ROS2 API Header */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

/* ROBOTIS Controller Header */
#include "robotis_controller/robotis_controller.h"

/* Sensor Module Header */
#include "open_cr_module/open_cr_module.h"

/* Motion Module Header */
#include "op3_base_module/base_module.h"
#include "op3_head_control_module/head_control_module.h"
#include "op3_action_module/action_module.h"
#include "op3_walking_module/op3_walking_module.h"
#include "op3_direct_control_module/direct_control_module.h"
#include "op3_online_walking_module/online_walking_module.h"
#include "op3_tuning_module/tuning_module.h"
#include "op3_walking_module_msgs/msg/walking_param.hpp"

using namespace robotis_framework;
using namespace dynamixel;
using namespace robotis_op;

std::string g_offset_file;
std::string g_robot_file;
std::string g_init_file;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_init_pose_pub;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr g_demo_command_pub;
rclcpp::Publisher<op3_walking_module_msgs::msg::WalkingParam>::SharedPtr walking_param_pub_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr walking_command_pub_;

rclcpp::Time last_cmd_time;
bool walking_active = false;

void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  constexpr double DEG_TO_RAD = M_PI / 180.0;
  constexpr double LINEAR_SCALE = 0.1;
  last_cmd_time = rclcpp::Clock().now();

  double lin_x = msg->linear.x;
  double lin_y = msg->linear.y;
  double lin_z = msg->linear.z;
  double ang_z = msg->angular.z;

  op3_walking_module_msgs::msg::WalkingParam param_msg;

  param_msg.x_move_amplitude = lin_x * LINEAR_SCALE;
  param_msg.y_move_amplitude = lin_y * LINEAR_SCALE;
  // param_msg.z_move_amplitude = lin_z * LINEAR_SCALE;
  param_msg.z_move_amplitude = 0.06; // constant for now
  param_msg.angle_move_amplitude = ang_z;

  param_msg.init_x_offset = -0.020;
  param_msg.init_y_offset = 0.015;
  param_msg.init_z_offset = 0.035;
  param_msg.init_roll_offset = 0.0;
  param_msg.init_pitch_offset = 0.0;
  param_msg.init_yaw_offset = 0.0 ;
  param_msg.hip_pitch_offset = 5.0 * DEG_TO_RAD;

  param_msg.period_time = 0.650;
  param_msg.dsp_ratio = 0.20;
  param_msg.step_fb_ratio = 0.28;

  param_msg.move_aim_on = false;
  param_msg.balance_enable = false;
  param_msg.balance_hip_roll_gain = 0.35;
  param_msg.balance_knee_gain = 0.30;
  param_msg.balance_ankle_roll_gain = 0.70;
  param_msg.balance_ankle_pitch_gain = 0.90;

  param_msg.y_swap_amplitude = 0.028;
  param_msg.z_swap_amplitude = 0.006;
  param_msg.arm_swing_gain = 0.20;
  param_msg.pelvis_offset = 0.0;

  param_msg.p_gain = 0;
  param_msg.i_gain = 0;
  param_msg.d_gain = 0;

  walking_param_pub_->publish(param_msg);

  if (!walking_active)
  {
    std_msgs::msg::String cmd_msg;
    cmd_msg.data = "start";
    walking_command_pub_->publish(cmd_msg);
    walking_active = true;

    RCLCPP_INFO(rclcpp::get_logger("cmd_vel_subscriber"), "Walking started");
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("op3_manager");

  auto timer = node->create_wall_timer(std::chrono::milliseconds(200), [node]() {
  static rclcpp::Time last_checked = rclcpp::Clock().now();
  rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

  if (walking_active && (rclcpp::Clock().now() - last_cmd_time) > timeout)
    {
      std_msgs::msg::String cmd_msg;
      cmd_msg.data = "stop";
      walking_command_pub_->publish(cmd_msg);
      walking_active = false;

      RCLCPP_INFO(node->get_logger(), "Walking stopped due to timeout");
    }
  });
  
  auto cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
  "/cmd_vel", 10, cmdVelCallback);

  RCLCPP_INFO(node->get_logger(), "manager->init");
  RobotisController *controller = RobotisController::getInstance();

  /* Load ROS Parameter */

  node->declare_parameter<std::string>("offset_file_path", "");
  node->declare_parameter<std::string>("robot_file_path", "");
  node->declare_parameter<std::string>("init_file_path", "");
  node->declare_parameter<int>("team_number", 0);
  node->declare_parameter<int>("player_number", 1);

  node->get_parameter("offset_file_path", g_offset_file);
  node->get_parameter("robot_file_path", g_robot_file);
  node->get_parameter("init_file_path", g_init_file);

  g_init_pose_pub = node->create_publisher<std_msgs::msg::String>("/robotis/base/ini_pose", 10);
  g_demo_command_pub = node->create_publisher<std_msgs::msg::String>("/ball_tracker/command", 10);
  walking_param_pub_ = node->create_publisher<op3_walking_module_msgs::msg::WalkingParam>("/robotis/walking/set_params", 10);
  walking_command_pub_ = node->create_publisher<std_msgs::msg::String>("/robotis/walking/command", 10);
  
  bool simulation = false;
  node->declare_parameter<bool>("simulation", false);
  node->get_parameter("simulation", simulation);
  // using flag consistent with ROBOTIS-OP3.op3_manager package
  controller->gazebo_mode_ = simulation;

  if (simulation) {
    RCLCPP_WARN(node->get_logger(), "Simulation Mode: ON");
  } else {
    RCLCPP_WARN(node->get_logger(), "Simulation Mode: OFF");
  }

  std::string robot_name;
  node->declare_parameter<std::string>("simulation_robot_name", "");
  node->get_parameter("simulation_robot_name", robot_name);
  if (robot_name != "")
    controller->gazebo_robot_name_ = robot_name;

  if (g_robot_file.empty())
  {  
    RCLCPP_ERROR(node->get_logger(), "NO robot file path in the ROS parameters.");
    return -1;
  }

  // initialize robot
  // in sim: pass empty init file so framework does not touch /dev/ttyUSB
  const std::string init_for_mode = controller->gazebo_mode_ ? "" : g_init_file;
  RCLCPP_INFO(node->get_logger(),
            "Initializing with robot_file='%s', init_file='%s'",
            g_robot_file.c_str(), init_for_mode.c_str());


  if (controller->initialize(g_robot_file, init_for_mode) == false)
  {
    RCLCPP_ERROR(node->get_logger(), "ROBOTIS Controller Initialize Fail!");
    return -1;
  }

  // load offset but not in simulation mode
  if (!controller->gazebo_mode_ && !g_offset_file.empty())
    controller->loadOffset(g_offset_file);

  rclcpp::sleep_for(std::chrono::milliseconds(300));

  /* Add Sensor Module */
  // controller->addSensorModule((SensorModule*) OpenCRModule::getInstance());

  /* Add Motion Module */
  // controller->addMotionModule((MotionModule*) ActionModule::getInstance());
  controller->addMotionModule((MotionModule*) BaseModule::getInstance());
  // controller->addMotionModule((MotionModule*) HeadControlModule::getInstance());
  controller->addMotionModule((MotionModule*) WalkingModule::getInstance());
  // controller->addMotionModule((MotionModule*) DirectControlModule::getInstance());
  // controller->addMotionModule((MotionModule*) OnlineWalkingModule::getInstance());
  // controller->addMotionModule((MotionModule*) TuningModule::getInstance());

  // start timer
  controller->startTimer();
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  // go to init pose
  std_msgs::msg::String init_msg;
  init_msg.data = "ini_pose";

  g_init_pose_pub->publish(init_msg);
  RCLCPP_INFO(node->get_logger(), "Controller; Base Module [initial pose]");
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  controller->setCtrlModule("walking_module");
  RCLCPP_INFO(node->get_logger(), "Controller: Walking Module");
  rclcpp::sleep_for(std::chrono::milliseconds(100));


  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
