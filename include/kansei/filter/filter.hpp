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

#ifndef KANSEI__FILTER__FILTER_HPP_
#define KANSEI__FILTER__FILTER_HPP_

#include <memory>
#include <string>

#include "./madgwick.hpp"
#include "../fallen_status.hpp"
#include "../measurement_unit.hpp"

namespace kansei
{

class Filter : public MeasurementUnit
{
public:
  Filter();

  void update_rpy();

  void reset_orientation();
  void set_orientation_to(const keisan::Angle<double> & target_orientation);
  void set_orientation_raw_to(double orientation);

  float get_rl_gyro() const {return gyro[0] - rl_gyro_center;}
  float get_fb_gyro() const {return gyro[1] - fb_gyro_center;}

  const bool & is_calibrated() const {return calibration_status;}

  void load_data(std::string path);

  double angle_compensation;
  double angle_raw_compensation;

private:
  MadgwickFilter filter;
  bool initialized;
  float last_seconds;

  double roll;
  double pitch;
  double yaw;
  double yaw_raw;

  double initial_yaw;
  double delta_yaw;
  bool init_yaw;

  double gyro[3];
  double gyro_mux[3];
  double rl_gyro_center;
  double fb_gyro_center;
  double rl_gyro_arr[100];
  double fb_gyro_arr[100];
  int rl_fb_gyro_counter;
};

}  // namespace kansei

#endif  // KANSEI__FILTER__FILTER_HPP_
