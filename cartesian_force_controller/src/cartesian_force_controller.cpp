////////////////////////////////////////////////////////////////////////////////
// Copyright 2019 FZI Research Center for Information Technology
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
// this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
// this list of conditions and the following disclaimer in the documentation
// and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
// contributors may be used to endorse or promote products derived from this
// software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

//-----------------------------------------------------------------------------
/*!\file    cartesian_force_controller.cpp
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#include <cartesian_force_controller/cartesian_force_controller.h>

#include <cmath>

#include "cartesian_controller_base/Utility.h"
#include "controller_interface/controller_interface.hpp"

namespace cartesian_force_controller
{
CartesianForceController::CartesianForceController()
: m_hand_frame_control(true)
{
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_init()
{
  const auto ret = Base::on_init();
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  auto_declare<std::string>("ft_sensor_ref_link", "");
  auto_declare<bool>("hand_frame_control", true);
  auto_declare<std::string>("ft_sensor_name", "ft_sensor");
  auto_declare<std::string>("ft_sensor_topic", "");

  return CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
CartesianForceController::state_interface_configuration() const
{
  // 获取基类的状态接口配置（关节位置）
  auto conf = Base::state_interface_configuration();
  
  // 检查是否配置了 topic 参数（如果配置了，自动使用 topic 模式）
  bool use_topic = false;
  try {
    if (get_node()->has_parameter("ft_sensor_topic")) {
      std::string topic = get_node()->get_parameter("ft_sensor_topic").as_string();
      use_topic = !topic.empty();
    }
  } catch (...) {
    // 如果参数不可用，使用默认值 false
  }
  
  // 只有在非 topic 模式下才需要硬件接口
  if (!use_topic) {
    // 从参数中读取传感器名称（如果可用），否则使用默认值
    std::string sensor_name = "ft_sensor";
    try {
      if (get_node()->has_parameter("ft_sensor_name")) {
        sensor_name = get_node()->get_parameter("ft_sensor_name").as_string();
      }
    } catch (...) {
      // 如果参数不可用，使用默认值
    }
    
    // 添加力传感器状态接口（使用配置的传感器名称）
    conf.names.push_back(sensor_name + "/force.x");
    conf.names.push_back(sensor_name + "/force.y");
    conf.names.push_back(sensor_name + "/force.z");
    conf.names.push_back(sensor_name + "/torque.x");
    conf.names.push_back(sensor_name + "/torque.y");
    conf.names.push_back(sensor_name + "/torque.z");
  }
  
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_configure(previous_state);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // 读取传感器名称配置
  m_ft_sensor_name = get_node()->get_parameter("ft_sensor_name").as_string();
  
  // 检查是否配置了 topic 参数（如果配置了，自动使用 topic 模式）
  std::string ft_sensor_topic = "";
  try {
    if (get_node()->has_parameter("ft_sensor_topic")) {
      ft_sensor_topic = get_node()->get_parameter("ft_sensor_topic").as_string();
    }
  } catch (...) {
    // 如果参数不可用，使用默认值
  }
  m_use_ft_sensor_topic = !ft_sensor_topic.empty();

  // Make sure sensor link is part of the robot chain
  m_ft_sensor_ref_link = get_node()->get_parameter("ft_sensor_ref_link").as_string();
  if (!robotChainContains(m_ft_sensor_ref_link))
  {
    RCLCPP_ERROR_STREAM(get_node()->get_logger(), m_ft_sensor_ref_link
                                                    << " is not part of the kinematic chain from "
                                                    << Base::m_robot_base_link << " to "
                                                    << Base::m_end_effector_link);
    return CallbackReturn::ERROR;
  }

  // Make sure sensor wrenches are interpreted correctly
  setFtSensorReferenceFrame(m_end_effector_link);

  m_target_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
    get_node()->get_name() + std::string("/target_wrench"), 10,
    std::bind(&CartesianForceController::targetWrenchCallback, this, std::placeholders::_1));

  // 如果使用 topic 模式，创建力传感器 topic subscriber
  if (m_use_ft_sensor_topic) {
    m_ft_sensor_wrench_subscriber = get_node()->create_subscription<geometry_msgs::msg::WrenchStamped>(
      ft_sensor_topic, 10,
      std::bind(&CartesianForceController::ftSensorWrenchCallback, this, std::placeholders::_1));
    RCLCPP_INFO(get_node()->get_logger(), 
                "Using topic mode for force-torque sensor: %s", ft_sensor_topic.c_str());
  }

  m_target_wrench.setZero();
  m_ft_sensor_wrench.setZero();

  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  const auto ret = Base::on_activate(previous_state);
  if (ret != CallbackReturn::SUCCESS)
  {
    return ret;
  }
  
  // 只有在非 topic 模式下才需要获取硬件接口
  if (!m_use_ft_sensor_topic) {
    // 获取力传感器状态接口句柄
    // 使用完整接口名称列表（与 state_interface_configuration() 中声明的格式一致）
    // 使用配置的传感器名称
    std::vector<std::string> ft_sensor_interface_names = {
      m_ft_sensor_name + "/force.x",
      m_ft_sensor_name + "/force.y",
      m_ft_sensor_name + "/force.z",
      m_ft_sensor_name + "/torque.x",
      m_ft_sensor_name + "/torque.y",
      m_ft_sensor_name + "/torque.z"
    };
    
    // 调试：打印所有可用的状态接口
    RCLCPP_INFO(get_node()->get_logger(), 
               "Available state interfaces: %zu", state_interfaces_.size());
    for (const auto& state_interface : state_interfaces_)
    {
      RCLCPP_INFO(get_node()->get_logger(), 
                 "  State interface: %s/%s", 
                 state_interface.get_name().c_str(),
                 state_interface.get_interface_name().c_str());
    }
    
    // 按照声明的顺序查找接口
    // 注意：对于传感器接口，get_name() 返回完整路径（如 "ft_sensor/force.x"），
    // 而 get_interface_name() 返回接口类型（如 "force.x"）
    m_ft_sensor_state_handles.clear();
    for (const auto& full_interface_name : ft_sensor_interface_names)
    {
      // 解析完整名称：格式为 "component_name/interface_name"
      size_t slash_pos = full_interface_name.find('/');
      if (slash_pos == std::string::npos) {
        RCLCPP_ERROR(get_node()->get_logger(), 
                     "Invalid interface name format: %s (expected 'component/interface')", 
                     full_interface_name.c_str());
        continue;
      }
      
      std::string interface_name = full_interface_name.substr(slash_pos + 1);
      
      // 查找匹配的接口
      // get_name() 返回完整路径（如 "ft_sensor/force.x"），get_interface_name() 返回接口类型（如 "force.x"）
      bool found = false;
      for (auto& state_interface : state_interfaces_)
      {
        // 检查完整名称是否匹配，或者检查接口类型是否匹配
        if (state_interface.get_name() == full_interface_name || 
            (state_interface.get_interface_name() == interface_name && 
             state_interface.get_name().find(m_ft_sensor_name) != std::string::npos))
        {
          m_ft_sensor_state_handles.push_back(std::ref(state_interface));
          RCLCPP_INFO(get_node()->get_logger(), 
                     "Found ft_sensor interface: %s/%s", 
                     state_interface.get_name().c_str(),
                     state_interface.get_interface_name().c_str());
          found = true;
          break;
        }
      }
      
      if (!found) {
        RCLCPP_WARN(get_node()->get_logger(), 
                   "Could not find interface: %s", full_interface_name.c_str());
      }
    }
    
    if (m_ft_sensor_state_handles.size() != 6)
    {
      RCLCPP_ERROR(get_node()->get_logger(), 
                   "Expected 6 ft_sensor state interfaces, got %zu.", 
                   m_ft_sensor_state_handles.size());
      RCLCPP_ERROR(get_node()->get_logger(), 
                   "Available state interfaces count: %zu", state_interfaces_.size());
      return CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Successfully activated with ft_sensor hardware interface");
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Successfully activated with ft_sensor topic mode");
  }
  
  return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
CartesianForceController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  Base::on_deactivate(previous_state);
  return CallbackReturn::SUCCESS;
}

void CartesianForceController::readFtSensorFromHardware()
{
  // 从 hardware interface 读取力传感器数据
  if (m_ft_sensor_state_handles.size() == 6)
  {
    KDL::Wrench tmp;
    tmp[0] = m_ft_sensor_state_handles[0].get().get_optional().value();  // force.x
    tmp[1] = m_ft_sensor_state_handles[1].get().get_optional().value();  // force.y
    tmp[2] = m_ft_sensor_state_handles[2].get().get_optional().value();  // force.z
    tmp[3] = m_ft_sensor_state_handles[3].get().get_optional().value();  // torque.x
    tmp[4] = m_ft_sensor_state_handles[4].get().get_optional().value();  // torque.y
    tmp[5] = m_ft_sensor_state_handles[5].get().get_optional().value();  // torque.z
    
    // 检查 NaN 值
    if (std::isnan(tmp[0]) || std::isnan(tmp[1]) || std::isnan(tmp[2]) ||
        std::isnan(tmp[3]) || std::isnan(tmp[4]) || std::isnan(tmp[5]))
    {
      auto & clock = *get_node()->get_clock();
      RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                  "NaN detected in force-torque sensor wrench from hardware interface. Ignoring input.");
    }
    else
    {
      // 计算测量到的力在目标参考系中的表示
      tmp = m_ft_sensor_transform * tmp;
      
      m_ft_sensor_wrench[0] = tmp[0];
      m_ft_sensor_wrench[1] = tmp[1];
      m_ft_sensor_wrench[2] = tmp[2];
      m_ft_sensor_wrench[3] = tmp[3];
      m_ft_sensor_wrench[4] = tmp[4];
      m_ft_sensor_wrench[5] = tmp[5];
    }
  }
}

controller_interface::return_type CartesianForceController::update(const rclcpp::Time & time,
                                                                   const rclcpp::Duration & period)
{
  // Synchronize the internal model and the real robot
  m_ik_solver->synchronizeJointPositions(m_joint_state_pos_handles);

  // 根据模式读取力传感器数据
  if (!m_use_ft_sensor_topic) {
    // 从 hardware interface 读取力传感器数据
    readFtSensorFromHardware();
  }
  // 如果使用 topic 模式，数据通过回调函数更新，不需要在这里读取

  // Control the robot motion in such a way that the resulting net force
  // vanishes.  The internal 'simulation time' is deliberately independent of
  // the outer control cycle.
  auto internal_period = rclcpp::Duration::from_seconds(0.02);

  // Compute the net force
  ctrl::Vector6D error = computeForceError();

  // Turn Cartesian error into joint motion
  computeJointControlCmds(error, internal_period);

  // Write final commands to the hardware interface
  writeJointControlCmds();

  return controller_interface::return_type::OK;
}

ctrl::Vector6D CartesianForceController::computeForceError()
{
  ctrl::Vector6D target_wrench;
  m_hand_frame_control = get_node()->get_parameter("hand_frame_control").as_bool();

  if (m_hand_frame_control)  // Assume end-effector frame by convention
  {
    target_wrench = displayInBaseLink(m_target_wrench, m_end_effector_link);
  }
  else  // Default to robot base frame
  {
    target_wrench = m_target_wrench;
  }

  // Superimpose target wrench and sensor wrench in base frame
  return displayInBaseLink(m_ft_sensor_wrench, m_new_ft_sensor_ref) + target_wrench;
}

void CartesianForceController::setFtSensorReferenceFrame(const std::string & new_ref)
{
  // Compute static transform from the force torque sensor to the new reference
  // frame of interest.
  m_new_ft_sensor_ref = new_ref;

  // Joint positions should cancel out, i.e. it doesn't matter as long as they
  // are the same for both transformations.
  KDL::JntArray jnts(m_ik_solver->getPositions());

  KDL::Frame sensor_ref;
  m_forward_kinematics_solver->JntToCart(jnts, sensor_ref, m_ft_sensor_ref_link);

  KDL::Frame new_sensor_ref;
  m_forward_kinematics_solver->JntToCart(jnts, new_sensor_ref, m_new_ft_sensor_ref);

  m_ft_sensor_transform = new_sensor_ref.Inverse() * sensor_ref;
}

void CartesianForceController::targetWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in target wrench. Ignoring input.");
    return;
  }

  m_target_wrench[0] = wrench->wrench.force.x;
  m_target_wrench[1] = wrench->wrench.force.y;
  m_target_wrench[2] = wrench->wrench.force.z;
  m_target_wrench[3] = wrench->wrench.torque.x;
  m_target_wrench[4] = wrench->wrench.torque.y;
  m_target_wrench[5] = wrench->wrench.torque.z;
}

void CartesianForceController::ftSensorWrenchCallback(
  const geometry_msgs::msg::WrenchStamped::SharedPtr wrench)
{
  if (!this->isActive())
  {
    return;
  }

  if (std::isnan(wrench->wrench.force.x) || std::isnan(wrench->wrench.force.y) ||
      std::isnan(wrench->wrench.force.z) || std::isnan(wrench->wrench.torque.x) ||
      std::isnan(wrench->wrench.torque.y) || std::isnan(wrench->wrench.torque.z))
  {
    auto & clock = *get_node()->get_clock();
    RCLCPP_WARN_STREAM_THROTTLE(get_node()->get_logger(), clock, 3000,
                                "NaN detected in force-torque sensor wrench. Ignoring input.");
    return;
  }

  KDL::Wrench tmp;
  tmp[0] = wrench->wrench.force.x;
  tmp[1] = wrench->wrench.force.y;
  tmp[2] = wrench->wrench.force.z;
  tmp[3] = wrench->wrench.torque.x;
  tmp[4] = wrench->wrench.torque.y;
  tmp[5] = wrench->wrench.torque.z;

  // Compute how the measured wrench appears in the frame of interest.
  tmp = m_ft_sensor_transform * tmp;

  m_ft_sensor_wrench[0] = tmp[0];
  m_ft_sensor_wrench[1] = tmp[1];
  m_ft_sensor_wrench[2] = tmp[2];
  m_ft_sensor_wrench[3] = tmp[3];
  m_ft_sensor_wrench[4] = tmp[4];
  m_ft_sensor_wrench[5] = tmp[5];
}


}  // namespace cartesian_force_controller

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(cartesian_force_controller::CartesianForceController,
                       controller_interface::ControllerInterface)
