// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "kansei/node/kansei_node.hpp"

#include "kansei/fallen/fallen.hpp"
#include "kansei/measurement/measurement.hpp"

using namespace std::chrono_literals;

namespace kansei
{

KanseiNode::KanseiNode(const std::string & node_name)
: rclcpp::Node(node_name), measurement_node(nullptr), fallen_node(nullptr)
{
  node_timer = this->create_wall_timer(
    8ms,
    [this]() {
      if (measurement_node) {
        measurement_node->update_measurement();

        if (fallen_node) {
          auto measurement_unit = measurement_node->get_measurement_unit();

          fallen_node->update_fallen(
            measurement_unit->get_orientation(), measurement_unit->get_filtered_acc());
        }
      }
    }
  );
}

void KanseiNode::set_measurement_unit(std::shared_ptr<MeasurementUnit> measurement_unit)
{
  measurement_node = std::make_shared<MeasurementNode>(
    this->create_sub_node(
      "measurement"), measurement_unit);
}

void KanseiNode::set_fallen_determinant(std::shared_ptr<FallenDeterminant> fallen_determinant)
{
  fallen_node = std::make_shared<FallenNode>(this->create_sub_node("fallen"), fallen_determinant);
}

}  // namespace kansei
