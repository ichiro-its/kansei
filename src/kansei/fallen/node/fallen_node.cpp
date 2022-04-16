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

#include <kansei_interfaces/msg/fallen.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "kansei/fallen/fallen.hpp"
#include "keisan/keisan.hpp"

using namespace keisan::literals;  // NOLINT

namespace kansei::fallen
{

FallenNode::FallenNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<FallenDeterminant> fallen_determinant)
: fallen_determinant(fallen_determinant)
{
  fallen_publisher = node->create_publisher<kansei_interfaces::msg::Fallen>(
    get_node_prefix() + "/fallen", 10);
}

void FallenNode::update(const keisan::Euler<double> & rpy, const keisan::Vector<3> & acc)
{
  if (fallen_determinant->get_determinant_type() == DeterminantType::ACCELERO) {
    fallen_determinant->update_fallen_status(acc);
  } else if (fallen_determinant->get_determinant_type() == DeterminantType::ORIENTATION) {
    fallen_determinant->update_fallen_status(rpy);
  } else {
    // do some exception
  }

  if (fallen_determinant->is_fallen()) {
    publish_fallen();
  }
}

std::string FallenNode::get_node_prefix() const
{
  return "fallen";
}

void FallenNode::publish_fallen()
{
  auto fallen_msg = kansei_interfaces::msg::Fallen();

  fallen_msg.fallen_status = fallen_determinant->get_fallen_status();

  fallen_publisher->publish(fallen_msg);
}

}  // namespace kansei::fallen
