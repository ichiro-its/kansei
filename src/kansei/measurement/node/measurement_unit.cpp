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

#include "kansei/measurement/node/measurement_unit.hpp"

#include "keisan/keisan.hpp"

using keisan::literals::operator""_deg;

namespace kansei::measurement
{

MeasurementUnit::MeasurementUnit()
: rpy(0_deg, 0_deg, 0_deg), calibrated(false), filtered_acc(keisan::Vector<3>::zero()),
  raw_acc(keisan::Vector<3>::zero()), gy(0_deg, 0_deg, 0_deg),
  acc(keisan::Point3::zero()), raw_gy(keisan::Vector<3>::zero()),
  filtered_acc_counter(0), filtered_gy_counter(0), start_button(0), stop_button(0),
  need_update_led(false), led_mode(0)
{
  for (int i = 0; i < 3; i++) {
    filtered_gy_center[i] = 512.0;
    filtered_acc[i] = 512.0;

    for (int j = 0; j < 100; j++) {
      filtered_gy_arr[i][j] = filtered_gy_center[i];

      if (j < 15) {
        filtered_acc_arr[i][j] = filtered_acc[i];
      }
    }
  }
}

void MeasurementUnit::update_gy_acc(
  const keisan::Vector<3> & gy, const keisan::Vector<3> & acc)
{
  this->raw_gy = gy;
  this->raw_acc = acc;

  if (!calibrated) {
    if (filtered_gy_counter < 100) {
      for (int i = 0; i < 3; i++) {
        filtered_gy_arr[i][filtered_gy_counter] = gy[i];
      }

      filtered_gy_counter++;
    } else {
      double filtered_gy_sum[3];
      for (int i = 0; i < 3; i++) {
        filtered_gy_sum[i] = 0.0;

        for (int j = 0; j < filtered_gy_counter; j++) {
          filtered_gy_sum[i] += filtered_gy_arr[i][j];
        }
      }

      double filtered_gy_mean[3];
      for (int i = 0; i < 3; i++) {
        filtered_gy_mean[i] = filtered_gy_sum[i] / filtered_gy_counter;
        filtered_gy_sum[i] = 0.0;
      }

      double filtered_gy_sd[3];
      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < filtered_gy_counter; j++) {
          filtered_gy_sum[i] += pow((filtered_gy_arr[i][j] - filtered_gy_mean[i]), 2);
        }

        filtered_gy_sd[i] = pow((filtered_gy_sum[i] / filtered_gy_counter), 0.5);
      }

      if (filtered_gy_sd[0] < 2.0 && filtered_gy_sd[1] < 2.0) {
        for (int i = 0; i < 3; i++) {
          filtered_gy_center[i] = filtered_gy_mean[i];
        }

        calibrated = true;
      }

      filtered_gy_counter = 0;
    }
  }

  if (calibrated) {
    if (filtered_acc_counter < 15) {
      for (int i = 0; i < 3; i++) {
        filtered_acc_arr[i][filtered_acc_counter] = acc[i];
      }

      filtered_acc_counter++;
    } else {
      double filtered_acc_sum[3];
      for (int i = 0; i < 3; i++) {
        filtered_acc_sum[i] = 0.0;

        for (int j = 0; j < filtered_acc_counter; j++) {
          filtered_acc_sum[i] += filtered_acc_arr[i][j];
        }
      }

      for (int i = 0; i < 3; i++) {
        filtered_acc[i] = filtered_acc_sum[i] / filtered_acc_counter;
      }

      filtered_acc_counter = 0;
    }

    for (int i = 0; i < 3; i++) {
      filtered_gy[i] = gy[i] / filtered_gy_center[i];
    }
  }
}

keisan::Euler<double> MeasurementUnit::get_orientation() const
{
  return keisan::Euler<double>(
    rpy.roll.normalize(), rpy.pitch.normalize(), rpy.yaw.normalize());
}

keisan::Vector<3> MeasurementUnit::get_filtered_gy() const
{
  return filtered_gy;
}

keisan::Vector<3> MeasurementUnit::get_filtered_acc() const
{
  return filtered_acc;
}

bool MeasurementUnit::is_calibrated() const
{
  return calibrated;
}

void MeasurementUnit::set_led(int mode) {
  need_update_led = true;
  led_mode = mode;
}

}  // namespace kansei::measurement
