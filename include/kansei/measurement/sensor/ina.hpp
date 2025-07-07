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

#ifndef KANSEI__MEASUREMENT__SENSOR__INA_HPP_
#define KANSEI__MEASUREMENT__SENSOR__INA_HPP_

#include <string>

#include "kansei/measurement/node/measurement_unit.hpp"
#include "keisan/angle.hpp"

namespace kansei::measurement
{

class INA : public MeasurementUnit
{
public:
  explicit INA(const std::string & port_name);
  ~INA();

  void set_port_name(const std::string & port_name);
  bool connect();

  void update_voltage() override;
  static void *start(void *object);

private:
  int socket_fd;
  std::string port_name;
  pthread_t thread;
};

}  // namespace kansei::measurement

#endif  // KANSEI__MEASUREMENT__SENSOR__INA_HPP_
