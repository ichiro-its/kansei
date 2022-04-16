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

#include <iomanip>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include "kansei/measurement/measurement.hpp"
#include "kansei/fallen/fallen.hpp"
#include "kansei/node/kansei_node.hpp"
#include "keisan/keisan.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string port_name = "/dev/ttyUSB1";

  if (argc > 1) {
    port_name = argv[1];
  }

  std::cout << "set the port name as " << port_name << "\n";
  auto mpu = std::make_shared<kansei::measurement::MPU>(port_name);

  std::cout << "connect to mpu\n";
  if (mpu->connect()) {
    std::cout << "succeeded to connect to mpu!\n";
  } else {
    std::cout << "failed to connect to mpu!\n" <<
      "try again!\n";
    return 0;
  }

  auto node = std::make_shared<rclcpp::Node>("kansei_node");
  auto kansei_node = std::make_shared<kansei::KanseiNode>(node);

  auto fallen = std::make_shared<kansei::fallen::FallenDeterminant>(
    kansei::fallen::DeterminantType::ACCELERO);

  kansei_node->set_measurement_unit(mpu);
  kansei_node->set_fallen_determinant(fallen);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
