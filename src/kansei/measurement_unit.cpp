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

#include "kansei/measurement_unit.hpp"

#include "kansei/fallen_status.hpp"

#include "keisan/keisan.hpp"

using namespace keisan::literals;  // NOLINT

namespace kansei
{

MeasurementUnit::MeasurementUnit()
  : rpy(0_deg, 0_deg, 0_deg), fallen_status(FallenStatus::STANDUP),
    is_calibrated(false), gy(keisan::Vector<3>::zero()), acc(keisan::Vector<3>::zero()),
    seconds(0.0), raw_acc_roll(512.0), raw_acc_pitch(482.0), raw_rp_counter(0),
    fallen_back_raw_limit(491.0), fallen_front_raw_limit(458.0),
    fallen_right_raw_limit(519.0), fallen_left_raw_limit(498.0)
{
  for (int i = 0; i < 15; i++) {
    raw_acc_roll_arr[i] = 512.0;
    raw_acc_pitch_arr[i] = 512.0;
  }
}

void MeasurementUnit::update_gy_acc(keisan::Vector<3> gy, keisan::Vector<3> acc, double seconds)
{
  this->gy = gy;
  this->acc = acc;

  delta_seconds = seconds - this->seconds;
  this->seconds = seconds;

  if (!is_calibrated) {
    if (raw_gy_rp_counter < 100) {
      raw_gy_roll_arr[raw_gy_rp_counter] = gy[0];
      raw_gy_pitch_arr[raw_gy_rp_counter] = gy[1];
      raw_gy_rp_counter++;
    } else {
      raw_gy_rp_counter = 0;

      double raw_gy_pitch_sum = 0.0;
      double raw_gy_roll_sum = 0.0;
      for (int i = 0; i < 100; i++) {
        raw_gy_pitch_sum += raw_gy_pitch_arr[i];
        raw_gy_roll_sum += raw_gy_roll_arr[i];
      }

      double pitch_mean = raw_gy_pitch_sum / 100;
      double roll_mean = raw_gy_roll_sum / 100;
      raw_gy_pitch_sum = 0.0;
      raw_gy_roll_sum = 0.0;
      for (int i = 0; i < 100; i++) {
        raw_gy_pitch_sum += pow((raw_gy_pitch_arr[i] - pitch_mean), 2);
        raw_gy_roll_sum += pow((raw_gy_roll_arr[i] - roll_mean), 2);
      }

      double raw_gy_pitch_sd = pow((raw_gy_pitch_sum / 100), 0.5);
      double raw_gy_roll_sd = pow((raw_gy_roll_sum / 100), 0.5);
      if (raw_gy_pitch_sd < 2.0 && raw_gy_roll_sd < 2.0) {
        raw_gy_pitch_center = pitch_mean;
        raw_gy_roll_center = roll_mean;
        is_calibrated = true;
      } else {
        raw_gy_pitch_center = 512.0;
        raw_gy_roll_center = 512.0;
      }
    }
  }

  if (is_calibrated) {
    if (raw_rp_counter < 15) {
      raw_acc_roll_arr[raw_rp_counter] = acc[0];
      raw_acc_pitch_arr[raw_rp_counter] = acc[1];
      raw_rp_counter++;
    } else {
      raw_rp_counter = 0;

      double raw_acc_roll_sum = 0.0;
      double raw_acc_pitch_sum = 0.0;
      for (int i = 0; i < 15; i++) {
        raw_acc_roll_sum += raw_acc_roll_arr[i];
        raw_acc_pitch_sum += raw_acc_pitch_arr[i];
      }

      raw_acc_roll = raw_acc_roll_sum / 15.0;
      raw_acc_pitch = raw_acc_pitch_sum / 15.0;
    }
  }
}

void MeasurementUnit::update_fallen_status()
{
  fallen_status = FallenStatus::STANDUP;

  if (raw_acc_pitch < fallen_front_raw_limit) {
    fallen_status = FallenStatus::FORWARD;
  } else if (raw_acc_pitch > fallen_back_raw_limit) {
    fallen_status = FallenStatus::BACKWARD;
  } else if (raw_acc_roll > fallen_right_raw_limit) {
    fallen_status = FallenStatus::RIGHT;
  } else if (raw_acc_roll < fallen_left_raw_limit) {
    fallen_status = FallenStatus::LEFT;
  }
}

keisan::Angle<double> MeasurementUnit::get_roll() const
{
  return rpy.roll;
}

keisan::Angle<double> MeasurementUnit::get_pitch() const
{
  return rpy.pitch;
}

keisan::Angle<double> MeasurementUnit::get_orientation() const
{
  return rpy.yaw.normalize();
}

bool MeasurementUnit::is_fallen() const
{
  return fallen_status != FallenStatus::STANDUP;
}

FallenStatus MeasurementUnit::get_fallen_status() const
{
  return fallen_status;
}

}  // namespace kansei
