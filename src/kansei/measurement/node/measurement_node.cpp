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

#include <experimental/array>
#include <array>
#include <memory>
#include <string>

#include "kansei_interfaces/msg/axis.hpp"
#include "kansei_interfaces/msg/unit.hpp"
#include "kansei/measurement/measurement.hpp"
#include "keisan/keisan.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace keisan::literals;  // NOLINT

namespace kansei
{

namespace measurement
{

MeasurementNode::MeasurementNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<MeasurementUnit> measurement_unit)
: measurement_unit(measurement_unit)
{
  orientation_publisher = node->create_publisher<kansei_interfaces::msg::Axis>(
    get_node_prefix() + "/orientation", 10);

  unit_publisher = node->create_publisher<kansei_interfaces::msg::Unit>(
    get_node_prefix() + "/unit", 10);

  // need to initialize some subscriber
}

void MeasurementNode::update_measurement()
{
  if (std::dynamic_pointer_cast<MPU>(measurement_unit)) {
    measurement_unit->update_rpy();

    publish_orientation();
  } else if (std::dynamic_pointer_cast<Filter>(measurement_unit)) {
    auto filter_measurement = std::dynamic_pointer_cast<Filter>(measurement_unit);

    subscribe_unit();

    // filter_measurement->update_gy_acc();
    filter_measurement->update_rpy();

    publish_orientation();
    publish_unit();
  } else {
    // do some exception
  }
}

std::shared_ptr<MeasurementUnit> MeasurementNode::get_measurement_unit() const
{
  return measurement_unit;
}

std::string MeasurementNode::get_node_prefix() const
{
  return "measurement";
}

void MeasurementNode::publish_orientation()
{
  auto orientation_msg = kansei_interfaces::msg::Axis();

  keisan::Euler<double> rpy = measurement_unit->get_orientation();

  orientation_msg.roll = rpy.roll.degree();
  orientation_msg.pitch = rpy.pitch.degree();
  orientation_msg.yaw = rpy.yaw.degree();

  orientation_publisher->publish(orientation_msg);
}

void MeasurementNode::publish_unit()
{
  auto unit_msg = kansei_interfaces::msg::Unit();

  keisan::Vector<3> gyro = measurement_unit->get_filtered_gy();
  keisan::Vector<3> accelero = measurement_unit->get_filtered_acc();

  unit_msg.gyro.roll = gyro[0];
  unit_msg.gyro.pitch = gyro[1];
  unit_msg.gyro.yaw = gyro[2];

  unit_msg.accelero.x = accelero[0];
  unit_msg.accelero.y = accelero[1];
  unit_msg.accelero.z = accelero[2];

  unit_publisher->publish(unit_msg);
}

void MeasurementNode::subscribe_unit()
{
  // do unit value subscribing
}

}  // namespace measurement

}  // namespace kansei