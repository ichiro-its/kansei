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

KanseiNode::KanseiNode(rclcpp::Node::SharedPtr node)
: node(node), measurement_node(nullptr), fallen_node(nullptr),
  start_seconds(node->now().seconds())
{
  node_timer = node->create_wall_timer(
    8ms,
    [this]() {
      if (this->measurement_node) {
        this->measurement_node->update(
          this->node->now().seconds() - this->start_seconds);

        if (this->fallen_node) {
          auto measurement_unit = this->measurement_node->get_measurement_unit();

          this->fallen_node->update(
            measurement_unit->get_orientation(), measurement_unit->get_filtered_acc());
        }
      }
    }
  );
}

void KanseiNode::set_measurement_unit(
  std::shared_ptr<measurement::MeasurementUnit> measurement_unit)
{
  measurement_node = std::make_shared<measurement::MeasurementNode>(
    node, measurement_unit);
}

void KanseiNode::set_fallen_determinant(
  std::shared_ptr<fallen::FallenDeterminant> fallen_determinant)
{
  fallen_node = std::make_shared<fallen::FallenNode>(
    node, fallen_determinant);
}

}  // namespace kansei
