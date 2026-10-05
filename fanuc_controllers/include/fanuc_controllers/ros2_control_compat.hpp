// Helpers bridging ros2_control API changes (Jazzy -> Lyrical).

#ifndef FANUC_CONTROLLERS__ROS2_CONTROL_COMPAT_HPP_
#define FANUC_CONTROLLERS__ROS2_CONTROL_COMPAT_HPP_

#include <limits>

#include "hardware_interface/loaned_state_interface.hpp"

namespace fanuc_controllers
{

// LoanedStateInterface::get_value() was removed; get_optional<double>() replaces it. An unavailable
// value maps to NaN, which is what an unset double state interface reads as.
inline double state_value(const hardware_interface::LoanedStateInterface& state_interface)
{
  return state_interface.get_optional<double>().value_or(std::numeric_limits<double>::quiet_NaN());
}

}  // namespace fanuc_controllers

#endif  // FANUC_CONTROLLERS__ROS2_CONTROL_COMPAT_HPP_
