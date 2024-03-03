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

  std::string port_name = "/dev/ttyUSB0";
  std::string fallen_type = "";
  std::string path = "";

  const char * help_message = 
    "Usage: ros2 run kansei main --path [config_path] --type [fallen_type]\n"
    "[config_path]:       path to the configuration file\n"
    "[fallen_type]:       fallen type to be used (orientation / accelero)\n"
    "Optional:\n"
    "-h, --help           show this help message and exit\n";

  if (argc > 1) {
    for (int i = 1; i < argc; i++) {
      std::string arg = argv[i];
      if (arg == "-h" || arg == "--help") {
        std::cout << help_message << std::endl;
        return 1;
      } else if (arg == "--path") {
        if (i + 1 < argc) {
          path = argv[i + 1];
          i++;
        } else {
          std::cerr << "Error: --path requires a path argument" << std::endl;
          return 1;
        }
      } else if (arg == "--type") {
        if (i + 1 < argc) {
          fallen_type = argv[i + 1];
          i++;
        } else {
          std::cerr << "Error: --type requires a fallen type argument" << std::endl;
          return 1;
        }
      }
    }
  } else {
    std::cout << "Invalid arguments!\n\n" << help_message << "\n";
    return 0;
  }

  auto mpu = std::make_shared<kansei::measurement::MPU>(port_name);

  if (mpu->connect()) {
    std::cout << "succeeded to connect to mpu " << port_name << "!\n";
  } else {
    port_name = "/dev/ttyUSB1";
    mpu->set_port_name(port_name);
    if (mpu->connect()) {
      std::cout << "succeeded to connect to mpu " << port_name << "!\n";
    } else {
      std::cout << "failed to connect to mpu!\n" <<
        "try again!\n";
      return 0;
    }
  }

  auto node = std::make_shared<rclcpp::Node>("kansei_node");
  auto kansei_node = std::make_shared<kansei::KanseiNode>(node);

  auto fallen = std::make_shared<kansei::fallen::FallenDeterminant>(fallen_type);
  fallen->load_config(path);

  kansei_node->set_measurement_unit(mpu);
  kansei_node->set_fallen_determinant(fallen);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
