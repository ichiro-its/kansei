// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <kansei_interfaces/msg/orientation.hpp>
#include <kansei_interfaces/msg/unit.hpp>
#include <rclcpp/rclcpp.hpp>

#include <array>

#include "kansei/measurement/node/measurement_unit.hpp"

#include "keisan/keisan.hpp"

using namespace keisan::literals;  // NOLINT

namespace kansei
{

MeasurementUnit::MeasurementUnit(rclcpp::Node::SharedPtr node)
: rpy(0_deg, 0_deg, 0_deg), is_calibrated(false), filtered_acc(keisan::Vector<3>::zero())
{
  orientation_publisher = node->create_publisher<kansei_interfaces::msg::Orientation>(
    get_node_prefix() + "/orientation", 10);

  unit_publisher = node->create_publisher<kansei_interfaces::msg::Unit>(
    get_node_prefix() + "/unit", 10);
}

keisan::Euler<double> MeasurementUnit::get_orientation() const
{
  return keisan::Euler<double>(
    rpy.roll.normalize(), rpy.pitch.normalize(), rpy.yaw.normalize());
}

keisan::Vector<3> MeasurementUnit::get_filtered_gy() const
{
  return filtered_gy;
}

keisan::Vector<3> MeasurementUnit::get_filtered_acc() const
{
  return filtered_acc;
}

void MeasurementUnit::publish_orientation()
{
  auto orientation_msg = kansei_interfaces::msg::Orientation();

  orientation_msg.orientation = std::array<float, 3>(
    {rpy.roll.normalize(), rpy.pitch.normalize(), rpy.yaw.normalize()});

  orientation_publisher->publish(orientation_msg);
}

void MeasurementUnit::publish_unit()
{
  auto unit_msg = kansei_interfaces::msg::Unit();

  unit_msg.gyro = std::array<float, 3>(
    {rpy.roll.normalize(), rpy.pitch.normalize(), rpy.yaw.normalize()});

  unit_publisher->publish(unit_msg);
}

void MeasurementUnit::subscribe_unit()
{

}

}  // namespace kansei
