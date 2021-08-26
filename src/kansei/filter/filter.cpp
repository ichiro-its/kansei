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

#include "kansei/filter/filter.hpp"

#include "kansei/filter/madgwick.hpp"
#include "keisan/keisan.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nlohmann/json.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

using namespace keisan::literals;  // NOLINT

namespace kansei
{

Filter::Filter()
: is_initialized(false), yaw_raw(0.0), gy_raw_mux(keisan::Vector<3>::zero()),
  orientation_compensation(0.0), raw_orientation_compensation(0.0)
{
  filter.set_world_frame(ENU);
  filter.set_algorithm_gain(0.1);
  filter.set_drift_bias_gain(0.0);
}

void Filter::update_rpy()
{
  for (int i = 0; i < 3; i++) {
    gy_raw[i] = ((gy[i] - 512.0) * (17.4532925199 / 1023.0)) * gy_raw_mux[i];
    acc_raw[i] = (acc[i] - 512.0) * (39.2266 / 512.0);
  }

  if (is_calibrated) {
    geometry_msgs::msg::Vector3 ang_vel;
    ang_vel.x = gy_raw[0];
    ang_vel.y = gy_raw[1];
    ang_vel.z = gy_raw[2];

    geometry_msgs::msg::Vector3 lin_acc;
    lin_acc.x = acc_raw[0];
    lin_acc.y = acc_raw[1];
    lin_acc.z = acc_raw[2];

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

    double temp_roll = 0.0;
    double temp_pitch = 0.0;
    double temp_yaw = 0.0;
    double q0 = 0.0;
    double q1 = 0.0;
    double q2 = 0.0;
    double q3 = 0.0;
    filter.get_orientation(q0, q1, q2, q3);
    tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0)).getRPY(temp_roll, temp_pitch, temp_yaw);

    if (temp_yaw < 0) {
      temp_yaw = (temp_yaw + keisan::pi<double>) * -1;
    } else if (temp_yaw >= 0) {
      temp_yaw = (temp_yaw - keisan::pi<double>) * -1;
    } else {
      temp_yaw = temp_yaw;
    }

    rpy.roll = keisan::make_degree(temp_roll * 180.0 / keisan::pi<double>);
    rpy.pitch = keisan::make_degree(temp_pitch * 180.0 / keisan::pi<double>);
    yaw_raw = keisan::make_degree(temp_yaw * 180.0 / keisan::pi<double>);
    rpy.yaw = yaw_raw + orientation_compensation;
  }
}

void Filter::load_data(std::string path)
{
  std::string file_name =
    path + "imu/" + "kansei.json";
  std::ifstream file(file_name);
  nlohmann::json imu_data = nlohmann::json::parse(file);

  for (const auto &[key, val] : imu_data.items()) {
    if (key == "filter") {
      try {
        val.at("gy_mux_x").get_to(gy_raw_mux[0]);
        val.at("gy_mux_y").get_to(gy_raw_mux[1]);
        val.at("gy_mux_z").get_to(gy_raw_mux[2]);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "fallen_limit") {
      try {
        val.at("fallen_back_limit").get_to(fallen_back_raw_limit);
        val.at("fallen_front_limit").get_to(fallen_front_raw_limit);
        val.at("fallen_right_limit").get_to(fallen_right_raw_limit);
        val.at("fallen_left_limit").get_to(fallen_left_raw_limit);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
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

}  // namespace kansei
