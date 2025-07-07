// Copyright (c) 2025 Ichiro ITS
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
#include <numeric>
#include <string>
#include <vector>

#include "kansei/measurement/measurement.hpp"
#include "keisan/keisan.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  std::string port_name = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0";

  if (argc > 1) {
    port_name = argv[1];
  }

  kansei::measurement::INA ina(port_name);

  if (ina.connect()) {
    std::cout << "succeeded to connect to ina " << port_name << "!\n";
  } else {
    std::cout << "failed to connect to ina!\n"
              << "try again!\n";
    return 0;
  }

  rclcpp::Rate loop_rate(100ms);
  while (rclcpp::ok()) {
    double pc_voltage = ina.get_pc_voltage();
    double pc_current = ina.get_pc_current();
    double servo_current = ina.get_servo_current();
    double servo_voltage = ina.get_servo_voltage();

    std::cout << "PC Voltage: " << pc_voltage << std::endl;
    std::cout << "PC Current: " << pc_current << std::endl;
    std::cout << "Servo Voltage: " << servo_voltage << std::endl;
    std::cout << "Servo Current: " << servo_current << std::endl;
    std::cout << "\033c";

    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
