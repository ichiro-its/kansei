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

namespace kansei::measurement
{

MeasurementNode::MeasurementNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<MeasurementUnit> measurement_unit)
: measurement_unit(measurement_unit)
{
  orientation_publisher = node->create_publisher<Axis>(
    get_node_prefix() + "/orientation", 10);

  reset_orientation_subscriber = node->create_subscription<ResetOrientation>(
    "/imu/unit", 10,
    [this](const ResetOrientation::SharedPtr message) {
      if (message->orientation == 0.0) {
        this->measurement_unit->reset_orientation();
      } else {
        this->measurement_unit->set_orientation_to(keisan::make_degree(message->orientation));
      }
    });

  unit_publisher = node->create_publisher<Unit>(
    get_node_prefix() + "/unit", 10);

  unit_subscriber = node->create_subscription<Unit>(
    "/imu/unit", 10,
    [this](const Unit::SharedPtr message) {
      keisan::Vector<3> gy(
        message->gyro.roll,
        message->gyro.pitch,
        message->gyro.yaw);
      keisan::Vector<3> acc(
        message->accelero.x,
        message->accelero.y,
        message->accelero.z);

      this->measurement_unit->update_gy_acc(gy, acc);
    });
}

void MeasurementNode::update(double seconds)
{
  if (std::dynamic_pointer_cast<MPU>(measurement_unit)) {
    measurement_unit->update_rpy();

    publish_orientation();
  } else if (std::dynamic_pointer_cast<Filter>(measurement_unit)) {
    auto filter_measurement = std::dynamic_pointer_cast<Filter>(measurement_unit);

    filter_measurement->update_seconds(seconds);
    filter_measurement->update_rpy();

    publish_orientation();
    publish_unit();
  } else {
    // TODO(maroqijalil): do some exception
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

  auto gyro = measurement_unit->get_filtered_gy();
  auto accelero = measurement_unit->get_filtered_acc();

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

}  // namespace kansei::measurement
