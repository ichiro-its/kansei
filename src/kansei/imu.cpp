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

#include "kansei/imu.hpp"

#include "kansei/filter/madgwick.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nlohmann/json.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

namespace kansei
{

Imu::Imu()
: initialized(false)
{
  filter.set_world_frame(ENU);
  filter.set_algorithm_gain(0.1);
  filter.set_drift_bias_gain(0.0);

  roll = 0.0;
  pitch = 0.0;
  yaw = 0.0;

  for (int i = 0; i < 3; i++) {
    gyro[i] = 0.0;
    gyro_mux[i] = 0.0;
    accelero[i] = 0.0;
  }

  rl_gyro_center = 512.0;
  fb_gyro_center = 512.0;

  for (int i = 0; i < 100; i++) {
    rl_gyro_arr[i] = rl_gyro_center;
    fb_gyro_arr[i] = fb_gyro_center;

    if (i < 15) {
      fb_accelero_arr[i] = fb_gyro_center;
      rl_accelero_arr[i] = rl_gyro_center;
    }
  }

  rl_fb_gyro_counter = 0;
  rl_fb_accelero_counter = 0;

  fallen_back_limit = 491.0;
  fallen_front_limit = 458.0;
  fallen_right_limit = 519.0;
  fallen_left_limit = 498.0;
  fallen_status = FallenStatus::STANDUP;

  print = true;
  calibration_status = false;

  delta_yaw = 0.0;
  init_yaw = true;

  angle_compensation = 0.0;

  min_fb = INFINITY;
  min_rl = INFINITY;
  max_fb = 0;
  max_rl = 0;

  fb_accelero = 482.0;
  rl_accelero = 512;
}

void Imu::compute_rpy(double gy[3], double acc[3], double seconds)
{
  for (int i = 0; i < 3; i++) {
    gyro[i] = ((gy[i] - 512.0) * (17.4532925199 / 1023.0)) * gyro_mux[i];
    accelero[i] = (acc[i] - 512.0) * (39.2266 / 512.0);
  }

  if (!calibration_status) {
    if (rl_fb_gyro_counter < 100) {
      fb_gyro_arr[rl_fb_gyro_counter] = gy[1];
      rl_gyro_arr[rl_fb_gyro_counter] = gy[0];
      rl_fb_gyro_counter++;
    } else {
      rl_fb_gyro_counter = 0;

      double fb_sum = 0.0;
      double rl_sum = 0.0;
      for (int i = 0; i < 100; i++) {
        fb_sum += fb_gyro_arr[i];
        rl_sum += rl_gyro_arr[i];
      }

      double fb_mean = fb_sum / 100;
      double rl_mean = rl_sum / 100;
      fb_sum = 0.0;
      rl_sum = 0.0;
      for (int i = 0; i < 100; i++) {
        fb_sum += pow((fb_gyro_arr[i] - fb_mean), 2);
        rl_sum += pow((rl_gyro_arr[i] - rl_mean), 2);
      }

      double fb_sd = pow((fb_sum / 100), 0.5);
      double rl_sd = pow((rl_sum / 100), 0.5);
      if (fb_sd < 2.0 && rl_sd < 2.0) {
        fb_gyro_center = fb_mean;
        rl_gyro_center = rl_mean;
        calibration_status = true;
      } else {
        fb_gyro_center = 512.0;
        rl_gyro_center = 512.0;
      }
    }
  }

  if (calibration_status) {
    if (rl_fb_accelero_counter < 15) {
      fb_accelero_arr[rl_fb_accelero_counter] = acc[0];
      rl_accelero_arr[rl_fb_accelero_counter] = acc[1];
      rl_fb_accelero_counter++;
    } else {
      rl_fb_accelero_counter = 0;

      double sum_fb = 0.0;
      double sum_rl = 0.0;
      for (int i = 0; i < 15; i++) {
        sum_fb += fb_accelero_arr[i];
        sum_rl += rl_accelero_arr[i];
      }

      fb_accelero = sum_fb / 15.0;
      rl_accelero = sum_rl / 15.0;

      if (min_fb > fb_accelero) {
        min_fb = fb_accelero;
      }

      if (max_fb < fb_accelero) {
        max_fb = fb_accelero;
      }

      if (min_rl > rl_accelero) {
        min_rl = rl_accelero;
      }

      if (max_rl < rl_accelero) {
        max_rl = rl_accelero;
      }
    }

    geometry_msgs::msg::Vector3 ang_vel;
    ang_vel.x = gyro[0];
    ang_vel.y = gyro[1];
    ang_vel.z = gyro[2];

    geometry_msgs::msg::Vector3 lin_acc;
    lin_acc.x = accelero[0];
    lin_acc.y = accelero[1];
    lin_acc.z = accelero[2];

    if (!initialized) {
      geometry_msgs::msg::Quaternion init_q;
      if (!StatelessOrientation::compute_orientation(ENU, lin_acc, init_q)) {
        return;
      }

      filter.set_orientation(init_q.w, init_q.x, init_q.y, init_q.z);
      last_seconds = seconds;
      initialized = true;
    }

    filter.madgwick_ahrs_update_imu(
      ang_vel.x, ang_vel.y, ang_vel.z,
      lin_acc.x, lin_acc.y, lin_acc.z,
      seconds - last_seconds);

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
      yaw = (temp_yaw + M_PI) * -1;
    } else if (temp_yaw >= 0) {
      yaw = (temp_yaw - M_PI) * -1;
    } else {
      yaw = temp_yaw;
    }

    yaw_raw = yaw * 180.0 / M_PI;

    last_seconds = seconds;
  }
}

FallenStatus Imu::get_fallen_status()
{
  fallen_status = FallenStatus::STANDUP;

  if (fb_accelero < fallen_front_limit) {
    fallen_status = FallenStatus::FORWARD;
  } else if (fb_accelero > fallen_back_limit) {
    fallen_status = FallenStatus::BACKWARD;
  } else if (rl_accelero > fallen_right_limit) {
    fallen_status = FallenStatus::RIGHT;
  } else if (rl_accelero < fallen_left_limit) {
    fallen_status = FallenStatus::LEFT;
  }

  return fallen_status;
}

void Imu::load_data(std::string path)
{
  std::string file_name =
    path + "imu/" + "kansei.json";
  std::ifstream file(file_name);
  nlohmann::json imu_data = nlohmann::json::parse(file);

  for (const auto &[key, val] : imu_data.items()) {
    if (key == "Imu") {
      try {
        val.at("gyro_mux_x").get_to(gyro_mux[0]);
        val.at("gyro_mux_y").get_to(gyro_mux[1]);
        val.at("gyro_mux_z").get_to(gyro_mux[2]);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "FallenLimit") {
      try {
        val.at("fallen_back_limit").get_to(fallen_back_limit);
        val.at("fallen_front_limit").get_to(fallen_front_limit);
        val.at("fallen_right_limit").get_to(fallen_right_limit);
        val.at("fallen_left_limit").get_to(fallen_left_limit);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    }
  }
}

void Imu::reset_orientation()
{
  reset_orientation_raw_to(0.0);
  reset_orientation_to(0.0);
}

void Imu::reset_orientation_to(double orientation)
{
  angle_compensation = orientation - (yaw_raw + angle_raw_compensation);
}

void Imu::reset_orientation_raw_to(double orientation)
{
  angle_raw_compensation = orientation - yaw_raw;
}

float Imu::get_yaw()
{
  double orientaion = (yaw * 180.0 / M_PI) + angle_compensation;

  if (orientaion < -180.0) {
    return orientaion + 360.0;
  } else if (orientaion >= 180.0) {
    return orientaion - 360.0;
  } else {
    return orientaion;
  }
}

}  // namespace kansei
