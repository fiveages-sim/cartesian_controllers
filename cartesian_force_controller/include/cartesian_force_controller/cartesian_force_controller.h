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
/*!\file    cartesian_force_controller.h
 *
 * \author  Stefan Scherzinger <scherzin@fzi.de>
 * \date    2017/07/27
 *
 */
//-----------------------------------------------------------------------------

#ifndef CARTESIAN_FORCE_CONTROLLER_H_INCLUDED
#define CARTESIAN_FORCE_CONTROLLER_H_INCLUDED

#include <cartesian_controller_base/ROS2VersionConfig.h>
#include <cartesian_controller_base/cartesian_controller_base.h>

#include <controller_interface/controller_interface.hpp>

#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace cartesian_force_controller
{
/**
 * @brief A ROS2-control controller for Cartesian force control
 *
 * This controller implements 6-dimensional end effector force control for
 * robots with a wrist force-torque sensor.  Users command
 * geometry_msgs::msg::WrenchStamped targets to steer the robot in task space.  The
 * controller additionally listens to the specified force-torque sensor signals
 * and computes the superposition with the target wrench.
 *
 * The underlying solver maps this remaining wrench to joint motion.
 * Users can steer their robot with this control in free space. The speed of
 * the end effector motion is set with PD gains on each Cartesian axes.
 * In contact, the controller regulates the net force of the two wrenches to zero.
 *
 * Note that during free motion, users can generally set higher control gains
 * for faster motion.  In contact with the environment, however, normally lower
 * gains are required to maintain stability.  The ranges to operate in mainly
 * depend on the stiffness of the environment and the controller cycle of the
 * real hardware, such that some experiments might be required for each use
 * case.
 *
 */
class CartesianForceController : public virtual cartesian_controller_base::CartesianControllerBase
{
public:
  CartesianForceController();

  CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration state_interface_configuration()
  const override;

  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(const rclcpp::Time & time,
                                           const rclcpp::Duration & period) override;

  using Base = CartesianControllerBase;

protected:
  /**
     * @brief Compute the net force of target wrench and measured sensor wrench
     *
     * @return The remaining error wrench, given in robot base frame
     */
  ctrl::Vector6D computeForceError();
  std::string m_new_ft_sensor_ref;
  void setFtSensorReferenceFrame(const std::string & new_ref);

  /**
     * @brief Read force-torque sensor data from hardware interface
     * 
     * This method reads the 6D force-torque data from the hardware interface
     * and updates m_ft_sensor_wrench. It should be called in the update() method
     * before computing the force error.
     */
  void readFtSensorFromHardware();

  // 力传感器状态接口句柄（从 hardware interface 读取）
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
  m_ft_sensor_state_handles;

  // 力传感器名称（从配置中读取）
  std::string m_ft_sensor_name;

  // 力传感器测量值（从 hardware interface 或 topic 读取后转换到目标参考系）
  ctrl::Vector6D m_ft_sensor_wrench;

  // 力传感器参考链接
  std::string m_ft_sensor_ref_link;

  // 力传感器变换矩阵
  KDL::Frame m_ft_sensor_transform;

  // 是否使用 topic 模式读取力传感器数据
  bool m_use_ft_sensor_topic = false;

private:
  void targetWrenchCallback(geometry_msgs::msg::WrenchStamped::SharedPtr wrench);
  void ftSensorWrenchCallback(geometry_msgs::msg::WrenchStamped::SharedPtr wrench);

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_target_wrench_subscriber;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr m_ft_sensor_wrench_subscriber;
  ctrl::Vector6D m_target_wrench;

  /**
     * Allow users to choose whether to specify their target wrenches in the
     * end-effector frame (= True) or the base frame (= False). The first one
     * is easier for explicit task programming, while the second one is more
     * intuitive for tele-manipulation.
     */
  bool m_hand_frame_control;
};
} // namespace cartesian_force_controller

#endif