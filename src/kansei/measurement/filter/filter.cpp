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

#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <string>

#include "kansei/measurement/filter/filter.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "jitsuyo/config.hpp"
#include "kansei/measurement/filter/madgwick/madgwick.hpp"
#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"

using namespace keisan::literals;  // NOLINT

namespace kansei::measurement
{

Filter::Filter()
: is_initialized(false), yaw_raw(keisan::make_degree(0.0)), raw_gy_mux(keisan::Vector<3>::zero()),
  orientation_compensation(keisan::make_degree(0.0)), seconds(0.0),
  raw_orientation_compensation(keisan::make_degree(0.0))
{
  filter.set_world_frame(ENU);
  filter.set_algorithm_gain(0.1);
  filter.set_drift_bias_gain(0.0);
}

void Filter::load_config(const std::string & path)
{
  nlohmann::json imu_data;
  if (!jitsuyo::load_config(path, "imu/kansei.json", imu_data)) {
    throw std::runtime_error("Failed to find config file");
  }
  
  bool valid_config = true;

  nlohmann::json filter_section;
  if (jitsuyo::assign_val(imu_data, "filter", filter_section)) {
    valid_config &= jitsuyo::assign_val(filter_section, "gyro_mux_x", raw_gy_mux[0]);
    valid_config &= jitsuyo::assign_val(filter_section, "gyro_mux_y", raw_gy_mux[1]);
    valid_config &= jitsuyo::assign_val(filter_section, "gyro_mux_z", raw_gy_mux[2]);
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file");
  }
}

void Filter::update_seconds(double seconds)
{
  delta_seconds = seconds - this->seconds;
  this->seconds = seconds;
}

void Filter::update_rpy()
{
  // value mapping (for conversion) refers to the link below:
  // https://emanual.robotis.com/docs/en/platform/op2/getting_started/
  gy.roll = keisan::make_degree(
    keisan::map(raw_gy[0], 512.0, 1023.0, 0.0, 8.72665) * raw_gy_mux[0]);
  gy.pitch = keisan::make_degree(
    keisan::map(raw_gy[1], 512.0, 1023.0, 0.0, 8.72665) * raw_gy_mux[1]);
  gy.yaw = keisan::make_degree(
    keisan::map(raw_gy[2], 512.0, 1023.0, 0.0, 8.72665) * raw_gy_mux[2]);

  acc.x = keisan::map(raw_acc[0], 512.0, 1023.0, 0.0, 39.2266);
  acc.y = keisan::map(raw_acc[1], 512.0, 1023.0, 0.0, 39.2266);
  acc.z = keisan::map(raw_acc[2], 512.0, 1023.0, 0.0, 39.2266);

  if (calibrated) {
    geometry_msgs::msg::Vector3 ang_vel;
    ang_vel.x = gy.roll.degree();
    ang_vel.y = gy.pitch.degree();
    ang_vel.z = gy.yaw.degree();

    geometry_msgs::msg::Vector3 lin_acc;
    lin_acc.x = acc.x;
    lin_acc.y = acc.y;
    lin_acc.z = acc.z;

    if (!is_initialized) {
      geometry_msgs::msg::Quaternion init_q;
      if (!StatelessOrientation::compute_orientation(ENU, lin_acc, init_q)) {
        return;
      }

      filter.set_orientation(init_q.w, init_q.x, init_q.y, init_q.z);
      is_initialized = true;
    }

    filter.madgwick_ahrs_update_imu(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      delta_seconds);

    auto temp_rpy = filter.get_orientation().euler();

    rpy.roll = keisan::make_degree(temp_rpy.roll.degree());
    rpy.pitch = keisan::make_degree(temp_rpy.pitch.degree());
    yaw_raw = keisan::make_degree(temp_rpy.yaw.normalize().degree());
    rpy.yaw = yaw_raw + orientation_compensation;
  }
}

void Filter::reset_orientation()
{
  set_orientation_raw_to(0_deg);
  set_orientation_to(0_deg);
}

void Filter::set_orientation_to(const keisan::Angle<double> & target_orientation)
{
  orientation_compensation = target_orientation - (yaw_raw + raw_orientation_compensation);
}

void Filter::set_orientation_raw_to(const keisan::Angle<double> & target_raw_orientation)
{
  raw_orientation_compensation = target_raw_orientation - yaw_raw;
}

}  // namespace kansei::measurement
