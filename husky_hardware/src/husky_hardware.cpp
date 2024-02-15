// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// ros2
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_mobile_base_hardware/hardware_info.hpp"

// local
#include "husky_hardware/husky_hardware.hpp"

namespace
{
const double MAXIMAL_SPEED = 1.0;
const double MAXIMAL_ACCELERATION = 5.0;
const std::string SERIAL_PORT = "/dev/prolific";
const double POLLING_TIMEOUT = 0.1;
}

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
HuskyHardware::HuskyHardware()
: HardwareSystemInterface<HardwareInterface4WD>(),
  front_wheel_radius_(0),
  rear_wheel_radius_(0),
  front_left_wheel_angular_speed_measure_(0),
  front_right_wheel_angular_speed_measure_(0),
  rear_left_wheel_angular_speed_measure_(0),
  rear_right_wheel_angular_speed_measure_(0),
  front_left_wheel_angular_speed_command_(0),
  front_right_wheel_angular_speed_command_(0),
  rear_left_wheel_angular_speed_command_(0),
  rear_right_wheel_angular_speed_command_(0)
{
}

//-----------------------------------------------------------------------------
HuskyHardware::~HuskyHardware()
{
  // force deactive when interface has not been deactivated by controller manager but by ctrl-c
  if (lifecycle_state_.id() == 3) {
    on_deactivate(lifecycle_state_);
  }
}


//-----------------------------------------------------------------------------
hardware_interface::return_type HuskyHardware::connect_()
{
  // RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Init communication with robot");
  // To be implememented
  horizon_legacy::connect(SERIAL_PORT);
  horizon_legacy::configureLimits(MAXIMAL_SPEED, MAXIMAL_ACCELERATION);

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type HuskyHardware::disconnect_()
{
  // RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Close communication with robot");
  // To be implememented

  send_null_command_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type HuskyHardware::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    // Get some info from ros2_control item of robot urdf file
    // wheelbase_ = get_parameter<double>(hardware_info, "wheelbase");
    // front_track_ = get_parameter<double>(hardware_info, "front_track");
    front_wheel_radius_ = get_parameter<float>(hardware_info, "front_wheel_radius");
    rear_wheel_radius_ = get_parameter<float>(hardware_info, "rear_wheel_radius");
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HuskyHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}

//-----------------------------------------------------------------------------
void HuskyHardware::send_null_command_()
{
  // Send null command to robot
  horizon_legacy::controlSpeed(0, 0, MAXIMAL_SPEED, MAXIMAL_SPEED);
}


//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type HuskyHardware::read()
#else
hardware_interface::return_type HuskyHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  // RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Read data from robot ");
  try {
    horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::Ptr speed =
      horizon_legacy::Channel<clearpath::DataDifferentialSpeed>::requestData(POLLING_TIMEOUT);

    if (speed) {
      front_left_wheel_angular_speed_measure_ = speed->getLeftSpeed() / front_wheel_radius_;
      rear_left_wheel_angular_speed_measure_ = speed->getLeftSpeed() / rear_wheel_radius_;
      front_right_wheel_angular_speed_measure_ = speed->getRightSpeed() / front_wheel_radius_;
      rear_right_wheel_angular_speed_measure_ = speed->getRightSpeed() / rear_wheel_radius_;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Could not get speed data");
    }

    set_hardware_state_();
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("HuskyHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}


//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type HuskyHardware::write()
# else
hardware_interface::return_type HuskyHardware::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  // RCLCPP_ERROR(rclcpp::get_logger("HuskyHardware"), "Send command to robot");
  get_hardware_command_();

  auto left_speed_command = front_left_wheel_angular_speed_command_ * front_wheel_radius_ +
    rear_left_wheel_angular_speed_command_ * rear_wheel_radius_;

  auto right_speed_command = front_right_wheel_angular_speed_command_ * front_wheel_radius_ +
    rear_right_wheel_angular_speed_command_ * rear_wheel_radius_;

  horizon_legacy::controlSpeed(
    left_speed_command, right_speed_command, MAXIMAL_SPEED, MAXIMAL_SPEED);

  return hardware_interface::return_type::OK;
}


//-----------------------------------------------------------------------------
void HuskyHardware::set_hardware_state_()
{
  core::HardwareState4WD state;
  state.frontLeftWheelSpinningMotion.velocity = front_left_wheel_angular_speed_measure_;
  state.frontRightWheelSpinningMotion.velocity = front_right_wheel_angular_speed_measure_;
  state.rearLeftWheelSpinningMotion.velocity = rear_left_wheel_angular_speed_measure_;
  state.rearRightWheelSpinningMotion.velocity = rear_right_wheel_angular_speed_measure_;
  this->hardware_interface_->set_state(state);
}

//-----------------------------------------------------------------------------
void HuskyHardware::get_hardware_command_()
{
  core::HardwareCommand4WD command = hardware_interface_->get_command();
  front_left_wheel_angular_speed_command_ = command.frontLeftWheelSpinningSetPoint;
  front_right_wheel_angular_speed_command_ = command.frontRightWheelSpinningSetPoint;
  rear_left_wheel_angular_speed_command_ = command.rearLeftWheelSpinningSetPoint;
  rear_right_wheel_angular_speed_command_ = command.rearRightWheelSpinningSetPoint;
}

}  // namespace ros2
}  // namespace romea


#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ros2::HuskyHardware, hardware_interface::SystemInterface)
