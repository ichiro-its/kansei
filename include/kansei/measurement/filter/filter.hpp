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

#ifndef KANSEI__MEASUREMENT__FILTER__FILTER_HPP_
#define KANSEI__MEASUREMENT__FILTER__FILTER_HPP_

#include <string>

#include "kansei/measurement/filter/madgwick/madgwick.hpp"
#include "kansei/measurement/node/measurement_unit.hpp"
#include "keisan/keisan.hpp"

namespace kansei::measurement
{

class Filter : public MeasurementUnit
{
public:
  Filter();

  void load_data(const std::string & path);

  void update_gy_acc(
    const keisan::Vector<3> & gy, const keisan::Vector<3> & acc,
    const double & seconds);
  void update_rpy() override;

  void reset_orientation() override;
  void set_orientation_to(const keisan::Angle<double> & target_orientation) override;
  void set_orientation_raw_to(const keisan::Angle<double> & target_raw_orientation);

private:
  MadgwickFilter filter;
  bool is_initialized;

  keisan::Angle<double> yaw_raw;
  keisan::Vector<3> raw_gy_mux;

  keisan::Angle<double> orientation_compensation;
  keisan::Angle<double> raw_orientation_compensation;

  // filter needs
  double seconds;
  double delta_seconds;

  // accelero variables
  double filtered_acc_arr[3][15];
  int filtered_acc_counter;
  // gyro variables
  double filtered_gy_arr[3][100];
  double filtered_gy_center[3];
  int filtered_gy_counter;
};

}  // namespace kansei::measurement

#endif  // KANSEI__MEASUREMENT__FILTER__FILTER_HPP_
