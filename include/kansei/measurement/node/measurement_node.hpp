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

#ifndef KANSEI__MEASUREMENT__NODE__MEASUREMENT_NODE_HPP_
#define KANSEI__MEASUREMENT__NODE__MEASUREMENT_NODE_HPP_

#include <memory>
#include <string>

#include "kansei/measurement/node/measurement_unit.hpp"
#include "kansei_interfaces/msg/axis.hpp"
#include "kansei_interfaces/msg/status.hpp"
#include "kansei_interfaces/msg/unit.hpp"
#include "kansei_interfaces/msg/power_status.hpp"
#include "tachimawari_interfaces/msg/status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

namespace kansei::measurement
{

class MeasurementNode
{
public:
  using Axis = kansei_interfaces::msg::Axis;
  using Float64 = std_msgs::msg::Float64;
  using Status = kansei_interfaces::msg::Status;
  using Unit = kansei_interfaces::msg::Unit;
  using ButtonStatus = tachimawari_interfaces::msg::Status;
  using PowerStatus = kansei_interfaces::msg::PowerStatus;

  static std::string get_node_prefix();
  static std::string reset_orientation_topic();
  static std::string status_topic();
  static std::string button_status_topic();
  static std::string power_status_topic();
  static std::string unit_topic();

  explicit MeasurementNode(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<MeasurementUnit> mpu_unit,
    std::shared_ptr<MeasurementUnit> ina_unit);

  void update(double seconds);

  std::shared_ptr<MeasurementUnit> get_mpu_unit() const;
  std::shared_ptr<MeasurementUnit> get_ina_unit() const;

private:
  void publish_status();
  void publish_unit();

  std::shared_ptr<MeasurementUnit> mpu_unit;
  std::shared_ptr<MeasurementUnit> ina_unit;

  rclcpp::Subscription<Float64>::SharedPtr reset_orientation_subscriber;

  rclcpp::Publisher<Unit>::SharedPtr unit_publisher;
  rclcpp::Subscription<Unit>::SharedPtr unit_subscriber;

  rclcpp::Publisher<Status>::SharedPtr status_publisher;

  rclcpp::Publisher<ButtonStatus>::SharedPtr button_status_publisher;

  rclcpp::Publisher<PowerStatus>::SharedPtr voltage_status_publisher;
};

}  // namespace kansei::measurement

#endif  // KANSEI__MEASUREMENT__NODE__MEASUREMENT_NODE_HPP_
