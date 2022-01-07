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

#ifndef KANSEI__FALLEN__NODE__FALLEN_NODE_HPP_
#define KANSEI__FALLEN__NODE__FALLEN_NODE_HPP_

#include <kansei_interfaces/msg/fallen.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "kansei/fallen/node/fallen_determinant.hpp"

namespace kansei
{

class FallenNode
{
public:
  explicit FallenNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<FallenDeterminant> fallen_determinant);

  std::string get_node_prefix() {return "fallen";}

  void update_fallen(keisan::Euler<double> rpy, keisan::Vector<3> acc);

protected:
  void publish_fallen();

  std::shared_ptr<FallenDeterminant> fallen_determinant;

  rclcpp::Publisher<kansei_interfaces::msg::Fallen> fallen_publisher;
};

}  // namespace kansei

#endif  // KANSEI__FALLEN__NODE__FALLEN_NODE_HPP_
