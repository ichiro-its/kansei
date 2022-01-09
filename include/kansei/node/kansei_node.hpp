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

#ifndef KANSEI__NODE__KANSEI_NODE_HPP_
#define KANSEI__NODE__KANSEI_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

#include "kansei/fallen/fallen.hpp"
#include "kansei/measurement/measurement.hpp"

namespace kansei
{

class KanseiNode : public rclcpp::Node
{
public:
  explicit KanseiNode(const std::string & node_name);

  void set_measurement_unit(std::shared_ptr<MeasurementUnit> measurement_unit);

  void set_fallen_determinant(std::shared_ptr<FallenDeterminant> fallen_determinant);

private:
  rclcpp::TimerBase::SharedPtr node_timer;

  std::shared_ptr<MeasurementNode> measurement_node;

  std::shared_ptr<FallenNode> fallen_node;
};

}  // namespace kansei

#endif  // KANSEI__NODE__KANSEI_NODE_HPP_
