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
#include <memory>
#include <string>

#include "kansei/measurement/measurement.hpp"
#include "keisan/keisan.hpp"

using namespace keisan::literals;  // NOLINT

namespace kansei
{

MeasurementNode::MeasurementNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<MeasurementUnit> measurement_unit)
: measurement_unit(measurement_unit)
{
  orientation_publisher = node->create_publisher<kansei_interfaces::msg::Orientation>(
    get_node_prefix() + "/orientation", 10);

  unit_publisher = node->create_publisher<kansei_interfaces::msg::Unit>(
    get_node_prefix() + "/unit", 10);

  // need to initialize some subscriber
}

void MeasurementNode::update_measurement()
{
  if (dynamic_cast<std::shared_ptr<MPU>>(measurement_unit)) {
    measurement_unit->update_rpy();

    publish_orientation();
  } else if (dynamic_cast<std::shared_ptr<Filter>>(measurement_unit)) {
    auto filter_measurement = dynamic_cast<std::shared_ptr<Filter>>(measurement_unit);

    subscribe_unit();

    // filter_measurement->update_gy_acc();
    filter_measurement->update_rpy();

    publish_orientation();
    publish_unit();
  } else {
    // do some exception
  }
}

void MeasurementNode::publish_orientation()
{
  auto orientation_msg = kansei_interfaces::msg::Orientation();

  keisan::Euler<double> rpy = unit->get_orientation();

  orientation_msg.orientation = std::array<float, 3>(
    {rpy.roll.normalize(), rpy.pitch.normalize(), rpy.yaw.normalize()});

  orientation_publisher->publish(orientation_msg);
}

void MeasurementNode::publish_unit()
{
  auto unit_msg = kansei_interfaces::msg::Unit();

  keisan::Vector<3> gy = unit->get_filtered_gy();
  keisan::Vector<3> acc = unit->get_filtered_acc();

  unit_msg.gyro = std::array<float, 3>({gy[0], gy[1], gy[2]});
  unit_msg.accelero = std::array<float, 3>({acc[0], acc[1], acc[2]});

  unit_publisher->publish(unit_msg);
}

void MeasurementNode::subscribe_unit()
{
  // do unit value subscribing
}

}  // namespace kansei
