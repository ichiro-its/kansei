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
#include "kansei_interfaces/msg/orientation.hpp"
#include "kansei_interfaces/msg/unit.hpp"
#include "rclcpp/rclcpp.hpp"

namespace kansei
{

namespace measurement
{

class MeasurementNode
{
public:
  explicit MeasurementNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<MeasurementUnit> measurement_unit);

  void update_measurement();

  std::shared_ptr<MeasurementUnit> get_measurement_unit() const;

protected:
  std::string get_node_prefix() const;

  void publish_orientation();

  void publish_unit();
  void subscribe_unit();

  std::shared_ptr<MeasurementUnit> measurement_unit;

  rclcpp::Publisher<kansei_interfaces::msg::Orientation>::SharedPtr orientation_publisher;

  rclcpp::Publisher<kansei_interfaces::msg::Unit>::SharedPtr unit_publisher;
  // need to declare some subscriber
};

}  // namespace measurement

}  // namespace kansei

#endif  // KANSEI__MEASUREMENT__NODE__MEASUREMENT_NODE_HPP_
