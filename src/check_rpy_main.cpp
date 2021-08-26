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
#include <numeric>
#include <string>
#include <vector>

#include "kansei/sensor/mpu.hpp"

int main(int argc, char * argv[])
{
  std::string port_name = "/dev/ttyUSB1";

  if (argc > 1) {
    port_name = argv[1];
  }

  std::cout << "set the port name as " << port_name << "\n";
  kansei::MPU mpu(port_name);

  std::cout << "connect to mpu\n";
  if (mpu.connect()) {
    std::cout << "succeeded to connect to mpu!\n";
  } else {
    std::cout << "failed to connect to mpu!\n" <<
      "try again!\n";
    return 0;
  }

  while (true) {
    mpu.update_rpy();

    std::cout << "Roll: " << mpu.get_roll().degree() << std::endl;
    std::cout << "Pitch: " << mpu.get_pitch().degree() << std::endl;
    std::cout << "Yaw: " << mpu.get_angle().degree() << std::endl;
    std::cout << "\033c";
  }

  return 0;
}
