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

#include "keisan/angle.hpp"

namespace kansei
{

void MeasurementUnit::update_gy_acc(keisan::Vector<3> gy, keisan::Vector<3> acc, double seconds)
{
  this->gy = gy;
  this->acc = acc;

  if (is_calibrated) {
    if (rp_counter < 15) {
      roll_arr[rp_counter] = acc[0];
      pitch_arr[rp_counter] = acc[1];
      rp_counter++;
    } else {
      rp_counter = 0;

      double sum_roll_arr = 0.0;
      double sum_pitch_arr = 0.0;
      for (int i = 0; i < 15; i++) {
        sum_roll_arr += roll_arr[i];
        sum_pitch_arr += pitch_arr[i];
      }

      roll_raw = sum_roll_arr / 15.0;
      pitch_raw = sum_pitch_arr / 15.0;
    }
  }
}

void MeasurementUnit::update_fallen_status()
{
  fallen_status = FallenStatus::STANDUP;

  if (pitch_raw < fallen_front_raw_limit) {
    fallen_status = FallenStatus::FORWARD;
  } else if (pitch_raw > fallen_back_raw_limit) {
    fallen_status = FallenStatus::BACKWARD;
  } else if (roll_raw > fallen_right_raw_limit) {
    fallen_status = FallenStatus::RIGHT;
  } else if (roll_raw < fallen_left_raw_limit) {
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
