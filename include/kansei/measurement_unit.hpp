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

#ifndef KANSEI__MEASUREMENT_UNIT_HPP_
#define KANSEI__MEASUREMENT_UNIT_HPP_

#include "./fallen_status.hpp"
#include "keisan/keisan.hpp"

namespace kansei
{

class MeasurementUnit
{
public:
  MeasurementUnit();
  virtual ~MeasurementUnit() {}

  virtual void update_rpy() {}
  void update_gy_acc(keisan::Vector<3> gy, keisan::Vector<3> acc, double seconds);

  void update_fallen_status();

  virtual void reset_orientation() {}
  virtual void set_orientation_to(const keisan::Angle<double> & target_orientation) {}

  keisan::Angle<double> get_roll() const;
  keisan::Angle<double> get_pitch() const;
  keisan::Angle<double> get_orientation() const;

  float get_roll_gy() const {return gy[0] - raw_gy_roll_center;}
  float get_pitch_gy() const {return gy[1] - raw_gy_pitch_center;}

  float get_roll_acc() const {return acc[0];}
  float get_pitch_acc() const {return acc[1];}

  bool is_fallen() const;
  FallenStatus get_fallen_status() const;

  keisan::EulerAngles rpy;

  FallenStatus fallen_status;

  bool is_calibrated;

  // filter needs
  keisan::Vector<3> gy;
  keisan::Vector<3> gy_raw;
  keisan::Vector<3> acc;
  keisan::Vector<3> acc_raw;
  double seconds;
  double delta_seconds;

  // fallen status changes needs
  // accelero variables
  double raw_acc_roll_arr[15];
  double raw_acc_roll;
  double raw_acc_pitch_arr[15];
  double raw_acc_pitch;
  int raw_acc_rp_counter;
  // gyro variables
  double raw_gy_roll_arr[100];
  double raw_gy_roll_center;
  double raw_gy_pitch_arr[100];
  double raw_gy_pitch_center;
  int raw_gy_rp_counter;
  // fallen raw variables
  float fallen_back_raw_limit;
  float fallen_front_raw_limit;
  float fallen_right_raw_limit;
  float fallen_left_raw_limit;
};

}  // namespace kansei

#endif  // KANSEI__MEASUREMENT_UNIT_HPP_
