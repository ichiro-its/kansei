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

#ifndef KANSEI__MEASUREMENT__NODE__MEASUREMENT_UNIT_HPP_
#define KANSEI__MEASUREMENT__NODE__MEASUREMENT_UNIT_HPP_

#include <string>

#include "keisan/keisan.hpp"

namespace kansei::measurement
{

class MeasurementUnit
{
public:
  MeasurementUnit();
  virtual ~MeasurementUnit() {}

  virtual void update_rpy() {}

  virtual void reset_orientation() {}
  virtual void set_orientation_to(const keisan::Angle<double> & target_orientation) {}

  keisan::Euler<double> get_orientation() const;

  void update_gy_acc(
    const keisan::Vector<3> & gy, const keisan::Vector<3> & acc);
  keisan::Vector<3> get_filtered_gy() const;
  keisan::Vector<3> get_filtered_acc() const;

protected:
  keisan::Euler<double> rpy;

  keisan::Vector<3> raw_gy;
  keisan::Euler<double> gy;
  keisan::Vector<3> filtered_gy;
  // gyro variables
  double filtered_gy_arr[3][100];
  double filtered_gy_center[3];
  int filtered_gy_counter;

  keisan::Vector<3> raw_acc;
  keisan::Point3 acc;
  keisan::Vector<3> filtered_acc;
  // accelero variables
  double filtered_acc_arr[3][15];
  int filtered_acc_counter;

  bool is_calibrated;
};

}  // namespace kansei::measurement

#endif  // KANSEI__MEASUREMENT__NODE__MEASUREMENT_UNIT_HPP_
