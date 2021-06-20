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
#include <kansei/complementary_filter.hpp>

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

  void reset_orientation();
  void reset_orientation_to(double orientation);
  void reset_orientation_raw_to(double orientation);

  void compute_rpy(double gy[3], double acc[3], double seconds);

  float get_roll() {return roll * 180.0/M_PI;}
  float get_pitch() {return pitch * 180.0/M_PI;}
  float get_yaw() {return (yaw * 180.0/M_PI) + angle_compensation;}

  float get_rl_gyro() {return gyro[0] - rl_gyro_center;}
  float get_fb_gyro() {return gyro[1] - fb_gyro_center;}

  bool is_fallen() {return fallen_status != FallenStatus::STANDUP;}
  FallenStatus get_fallen_status();

  bool is_calibrated() {return calibration_status;}

  void load_data(std::string path);

  double angle_compensation;
  double angle_raw_compensation;

private:
  // imu madgwick
  ImuFilter filter;
  bool initialized;
  float last_seconds;

  // imu madgwick
  ComplementaryFilter com_filter;
  bool com_initialized;

  double roll;
  double pitch;
  double yaw;
  double yaw_raw;

  double initial_yaw;
  double delta_yaw;
  bool init_yaw;

  double gyro[3];
  double gyro_mux[3];
  // double last_gyro_x;
  // double last_gyro_y;
  // double last_gyro_z;
  double rl_gyro_center;
  double fb_gyro_center;
  double rl_gyro_arr[100];
  double fb_gyro_arr[100];
  int rl_fb_gyro_counter;

  double accelero[3];
  double rl_accelero_arr[15];
  double fb_accelero_arr[15];
  double rl_accelero;
  double fb_accelero;
  int rl_fb_accelero_counter;

  float fallen_back_limit;
  float fallen_front_limit;
  float fallen_right_limit;
  float fallen_left_limit;
  FallenStatus fallen_status;

  bool print;
  bool calibration_status;
};

}  // namespace kansei

#endif  // KANSEI__IMU_HPP_
