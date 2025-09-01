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

#include <array>
#include <experimental/array>
#include <memory>
#include <string>

#include "kansei/measurement/measurement.hpp"
#include "kansei_interfaces/msg/axis.hpp"
#include "kansei_interfaces/msg/unit.hpp"
#include "keisan/keisan.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari/imu/node/imu_node.hpp"

using namespace keisan::literals;  // NOLINT

namespace kansei::measurement
{

std::string MeasurementNode::get_node_prefix() { return "measurement"; }

std::string MeasurementNode::reset_orientation_topic()
{
  return get_node_prefix() + "/reset_orientation";
}

std::string MeasurementNode::reset_pitch_topic()
{
  return get_node_prefix() + "/reset_pitch";
}

std::string MeasurementNode::status_topic() { return get_node_prefix() + "/status"; }

std::string MeasurementNode::button_status_topic() {  return get_node_prefix() + "/button_status"; }

std::string MeasurementNode::unit_topic() { return get_node_prefix() + "/unit"; }

std::string MeasurementNode::led_topic() { return get_node_prefix() + "/led"; }

MeasurementNode::MeasurementNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<MeasurementUnit> measurement_unit)
: measurement_unit(measurement_unit)
{
  reset_orientation_subscriber = node->create_subscription<Float64>(
    reset_orientation_topic(), 10, [this](const Float64::SharedPtr message) {
      if (message->data == 0.0) {
        this->measurement_unit->reset_orientation();
      } else {
        this->measurement_unit->set_orientation_to(keisan::make_degree(message->data));
      }
    });

  reset_pitch_subscriber = node->create_subscription<Empty>(
    reset_pitch_topic(), 10, [this](const Empty & message) {
      this->measurement_unit->reset_pitch();
    });

  unit_publisher = node->create_publisher<Unit>(unit_topic(), 10);

  status_publisher = node->create_publisher<Status>(status_topic(), 10);

  button_status_publisher = node->create_publisher<ButtonStatus>(button_status_topic(), 10);

  unit_subscriber = node->create_subscription<Unit>(
    tachimawari::imu::ImuNode::unit_topic(), 10, [this](const Unit::SharedPtr message) {
      keisan::Vector<3> gy(message->gyro.roll, message->gyro.pitch, message->gyro.yaw);
      keisan::Vector<3> acc(message->accelero.x, message->accelero.y, message->accelero.z);

      this->measurement_unit->update_gy_acc(gy, acc);
    });

  led_status_subscriber = node->create_subscription<Int8>(
    led_topic(), 10, [this](const Int8::SharedPtr message) {
      this->measurement_unit->set_led(message->data);
    });
}

void MeasurementNode::update(double seconds)
{
  if (std::dynamic_pointer_cast<MPU>(measurement_unit)) {

    publish_status();
  } else if (std::dynamic_pointer_cast<Filter>(measurement_unit)) {
    auto filter_measurement = std::dynamic_pointer_cast<Filter>(measurement_unit);

    filter_measurement->update_seconds(seconds);
    filter_measurement->update_rpy();

    publish_status();
    publish_unit();
  } else {
    // TODO(maroqijalil): do some exception
  }
}

std::shared_ptr<MeasurementUnit> MeasurementNode::get_measurement_unit() const
{
  return measurement_unit;
}

void MeasurementNode::publish_status()
{
  auto status_msg = Status();
  auto button_status_msg = ButtonStatus();
  button_status_msg.button = 0;

  status_msg.is_calibrated = measurement_unit->is_calibrated();

  keisan::Euler<double> rpy = measurement_unit->get_orientation();

  status_msg.orientation.roll = rpy.roll.degree();
  status_msg.orientation.pitch = rpy.pitch.degree();
  status_msg.orientation.yaw = rpy.yaw.degree();

  if (measurement_unit->start_button)
    button_status_msg.button = 2;

  else if (measurement_unit->stop_button)
    button_status_msg.button = 1;

  status_publisher->publish(status_msg);
  button_status_publisher->publish(button_status_msg);
}

void MeasurementNode::publish_unit()
{
  auto unit_msg = Unit();

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

}  // namespace kansei::measurement
