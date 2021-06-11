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

#ifndef KANSEI__IMU_HPP_
#define KANSEI__IMU_HPP_

#include <kansei/imu_filter.hpp>

#include <string>
#include <memory>

namespace kansei
{

enum FallenStatus
{
  LEFT,
  BACKWARD,
  STANDUP,
  FORWARD,
  RIGHT
};

class Imu
{
public:
  Imu();

  void compute_rpy(float gy[3], float acc[3], float seconds);

  float get_roll() {return roll;}
  float get_pitch() {return pitch;}
  float get_yaw() {return yaw;}

  float get_rl_gyro() {return gyro[0] - rl_gyro_center;}
  float get_fb_gyro() {return gyro[1] - fb_gyro_center;}

  bool is_fallen() {return fallen_status != FallenStatus::STANDUP;}
  FallenStatus get_fallen_status();

private:
  void load_data();

  ImuFilter filter;

  bool initialized;
  float last_seconds;

  double roll;
  double pitch;
  double yaw;

  double gyro[3];
  double accelero[3];

  double rl_gyro_center;
  double fb_gyro_center;

  float fallen_back_limit;
  float fallen_front_limit;
  float fallen_right_limit;
  float fallen_left_limit;
  FallenStatus fallen_status;
};

}  // namespace kansei

#endif  // KANSEI__IMU_HPP_
