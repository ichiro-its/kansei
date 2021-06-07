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

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <kansei/imu.hpp>
#include <kansei/stateless_orientation.hpp>

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <functional>

namespace kansei
{

Imu::Imu()
: initialized(false)
{
  filter.setWorldFrame(ENU);
  filter.setAlgorithmGain(0.1);
  filter.setDriftBiasGain(0.0);

  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;

  for (int i = 0; i < 3; i++) {
    gyro[i] = 0.0;
    accelero[i] = 0.0;
  }

  rl_gyro_center = 512.0;
  fb_gyro_center = 512.0;
}

void Imu::compute_rpy(float gy[3], float acc[3], float seconds)
{
  for (int i = 0; i < 3; i++) {
    gyro[i] = gy[i];
    accelero[i] = acc[i];
  }

  geometry_msgs::msg::Vector3 ang_vel;
  ang_vel.x = gy[0];
  ang_vel.y = gy[1];
  ang_vel.z = gy[2];

  geometry_msgs::msg::Vector3 lin_acc;
  lin_acc.x = acc[0];
  lin_acc.y = acc[1];
  lin_acc.z = acc[2];

  if (!initialized) {
    geometry_msgs::msg::Quaternion init_q;
    if (!StatelessOrientation::computeOrientation(ENU, lin_acc, init_q)) {
      return;
    }

    filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
    last_seconds = seconds;
    initialized = true;
  }

  filter.madgwickAHRSupdateIMU(
    ang_vel.x, ang_vel.y, ang_vel.z,
    lin_acc.x, lin_acc.y, lin_acc.z,
    seconds - last_seconds);
  last_seconds = seconds;

  double q0;
  double q1;
  double q2;
  double q3;
  filter.getOrientation(q0, q1, q2, q3);
  tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0)).getRPY(roll, pitch, yaw);
}

}  // namespace kansei
