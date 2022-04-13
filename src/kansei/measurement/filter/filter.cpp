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

void Filter::load_data(const std::string & path)
{
  std::string file_name =
    path + "imu/" + "kansei.json";
  std::ifstream file(file_name);
  nlohmann::json imu_data = nlohmann::json::parse(file);

  for (const auto &[key, val] : imu_data.items()) {
    if (key == "filter") {
      try {
        val.at("gy_mux_x").get_to(raw_gy_mux[0]);
        val.at("gy_mux_y").get_to(raw_gy_mux[1]);
        val.at("gy_mux_z").get_to(raw_gy_mux[2]);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
  }
}

void Filter::update_second(double seconds)
{
  delta_seconds = seconds - this->seconds;
  this->seconds = seconds;
}

void Filter::update_rpy()
{
  for (int i = 0; i < 3; i++) {
    // value mapping (for conversion) refers to the link below:
    // https://emanual.robotis.com/docs/en/platform/op2/getting_started/
    gy[i] = keisan::map(raw_gy[i], 512.0, 1023.0, 0.0, 8.72665) * raw_gy_mux[i];
    acc[i] = keisan::map(raw_acc[i], 512.0, 1023.0, 0.0, 39.2266);
  }

  if (is_calibrated) {
    geometry_msgs::msg::Vector3 ang_vel;
    ang_vel.x = gy[0];
    ang_vel.y = gy[1];
    ang_vel.z = gy[2];

    geometry_msgs::msg::Vector3 lin_acc;
    lin_acc.x = acc[0];
    lin_acc.y = acc[1];
    lin_acc.z = acc[2];

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
