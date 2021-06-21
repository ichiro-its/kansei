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

#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <kansei/imu.hpp>
#include <kansei/stateless_orientation.hpp>

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <functional>

namespace kansei
{

Imu::Imu()
: initialized(false)
{
  // imu madgwick
  filter.setWorldFrame(ENU);
  filter.setAlgorithmGain(0.1);
  filter.setDriftBiasGain(0.0);

  // imu complementary
  com_initialized = false;

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

  fallen_back_limit = 620.0;
  fallen_front_limit = 390.0;
  fallen_right_limit = 610.0;
  fallen_left_limit = 410.0;
  fallen_status = FallenStatus::STANDUP;

  print = true;
  calibration_status = false;

  delta_yaw = 0.0;
  init_yaw = true;

  angle_compensation = 0.0;
}

void Imu::compute_rpy(double gy[3], double acc[3], double seconds)
{
  for (int i = 0; i < 3; i++) {
    gyro[i] = ((gy[i] - 512.0) * (17.4532925199 / 1023.0)) * gyro_mux[i];
    accelero[i] = (acc[i] - 512.0) * (39.2266 / 512.0);
  }

  fb_accelero = acc[0];
  rl_accelero = acc[1];

  if (!calibration_status) {
    // std::cout << "calibrating" << std::endl;
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
      fb_accelero_arr[rl_fb_accelero_counter] = acc[1];
      rl_accelero_arr[rl_fb_accelero_counter] = acc[0];
      rl_fb_accelero_counter++;
    } else {
      rl_fb_accelero_counter = 0;
    }

    double sum_fb = 0.0;
    double sum_rl = 0.0;
    for (int i = 0; i < 15; i++) {
      sum_fb += fb_accelero_arr[i];
      sum_rl += rl_accelero_arr[i];
    }

    // int avr_fb = sum_fb / 15;
    // int avr_rl = sum_rl / 15;

    geometry_msgs::msg::Vector3 ang_vel;
    ang_vel.x = gyro[0];
    ang_vel.y = gyro[1];
    ang_vel.z = gyro[2];

    geometry_msgs::msg::Vector3 lin_acc;
    lin_acc.x = accelero[0];
    lin_acc.y = accelero[1];
    lin_acc.z = accelero[2];

    // double acc_angle_y = atan(-1*lin_acc.x/sqrt(pow(lin_acc.y,2) +
    // pow(lin_acc.z,2)))*(180.0/M_PI);
    // double acc_angle_x = atan(lin_acc.y/sqrt(pow(lin_acc.x,2) +
    // pow(lin_acc.z,2)))*(180.0/M_PI);
    // double acc_angle_z = 0;

    // double dt = seconds - last_seconds;

    // double gyro_angle_x = ang_vel.x*(180.0/M_PI)*dt + last_gyro_x;
    // double gyro_angle_y = ang_vel.y*(180.0/M_PI)*dt + last_gyro_y;
    // double gyro_angle_z = ang_vel.z*(180.0/M_PI)*dt + last_gyro_z;

    // double alpha = 0.96;
    // double angle_x = alpha*gyro_angle_x + (1.0-alpha)*acc_angle_x;
    // double angle_y = alpha*gyro_angle_y + (1.0-alpha)*acc_angle_y;
    // double angle_z = alpha*gyro_angle_z + (1.0-alpha)*acc_angle_z;

    // std::cout << "imu filter" << std::endl;
    // std::cout << "accx " << acc[0] << ", accy " << acc[1] << ", accz " << acc[2] << std::endl;
    // std::cout << "gyx " << gy[0] << ", gyy " << gy[1] << ", gyz " << gy[2] << std::endl;
    // std::cout << "ax " << lin_acc.x << ", ay " << lin_acc.y << ", az " << lin_acc.z << std::endl;
    // std::cout << "gx " << ang_vel.x << ", gy " << ang_vel.y << ", gz " << ang_vel.z << std::endl;
    // std::cout << "fb_accelero " << fb_accelero << ", rl_accelero " << rl_accelero << std::endl;

    if (!initialized) {
      geometry_msgs::msg::Quaternion init_q;
      if (!StatelessOrientation::computeOrientation(ENU, lin_acc, init_q)) {
        return;
      }

      filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
      last_seconds = seconds;
      initialized = true;
    }

    filter.madgwickAHRSupdateIMU(
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
    filter.getOrientation(q0, q1, q2, q3);
    tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0)).getRPY(temp_roll, temp_pitch, temp_yaw);

    if (temp_yaw < 0) {
      yaw = (temp_yaw + M_PI) * -1;
    } else if (temp_yaw >= 0) {
      yaw = (temp_yaw - M_PI) * -1;
    } else {
      yaw = temp_yaw;
    }

    yaw_raw = yaw * 180.0 / M_PI;
    // std::cout << "temp_yaw " << temp_yaw * 180.0/M_PI << std::endl;
    // std::cout << "yaw_raw " << yaw_raw << std::endl;
    // std::cout << "yaw_raw_comp " << angle_raw_compensation << std::endl;
    // std::cout << "yaw_raw " << yaw_raw + angle_raw_compensation << std::endl;

    // if (init_yaw) {
    //   initial_yaw = (yaw * 180.0/M_PI);
    //   init_yaw = false;
    // }

    // if (yaw < 0.0) {
    //   yaw += 180.0;
    // } else if (yaw >= 0.0) {
    //   yaw -= 180.0;
    // }

    // std::cout << "========= madgwick filter" << std::endl;
    // std::cout << "q0 " << q0 << ", q1 " << q1 << ", q2 " << q2 << ", q3 " << q3 << std::endl;
    // std::cout << "roll " << roll << ", temp_yaw " << temp_yaw << ", yaw " <<
    // yaw * 180.0/M_PI << std::endl;

    // delta_yaw += ((yaw * 180.0/M_PI) - initial_yaw) * 3.0;
    // initial_yaw = (yaw * 180.0/M_PI);
    // double value = std::fmod((std::fabs(initial_yaw) - std::fabs((yaw * 180.0/M_PI)))* 3.0,
    // 180.0);
    // double value = std::fmod((yaw * 180.0/M_PI+180), 90) * 4.0;
    // error = value / 4;
    // std::cout << "roll " << roll << ", pitch " << pitch << ", yaw " <<
    // value - (std::fmod((yaw * 180.0/M_PI+180), 90)) << std::endl;
    // std::cout << "roll " << roll << ", pitch " << pitch << ", yaw " <<
    // std::fmod(((initial_yaw) - std::fabs((yaw * 180.0/M_PI))) * 3.0/2.0, 360.0) << std::endl;

    // com_filter.update(
    //   lin_acc.x, lin_acc.y, lin_acc.z,
    //   ang_vel.x, ang_vel.y, ang_vel.z,
    //   seconds - last_seconds);

    // last_gyro_x = ang_vel.x * (180.0/M_PI);
    // last_gyro_y = ang_vel.y * (180.0/M_PI);
    // last_gyro_z = ang_vel.z * (180.0/M_PI);

    // roll = 0.0;
    // pitch = 0.0;
    // yaw = 0.0;
    // q0 = 0.0;
    // q1 = 0.0;
    // q2 = 0.0;
    // q3 = 0.0;
    // com_filter.getOrientation(q0, q1, q2, q3);
    // tf2::Matrix3x3(tf2::Quaternion(q1, q2, q3, q0)).getRPY(roll, pitch, yaw);

    // std::cout << "========= complementary filter" << std::endl;
    // std::cout << "q0 " << q0 << ", q1 " << q1 << ", q2 " << q2 << ", q3 " << q3 << std::endl;
    // std::cout << "roll " << roll << ", pitch " << pitch << ", yaw " << yaw * 180.0/M_PI <<
    // std::endl;
    // std::cout << "========================" << std::endl;

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
    if (key == "Madgwick") {
      try {
        val.at("gyro_mux_x").get_to(gyro_mux[0]);
        val.at("gyro_mux_y").get_to(gyro_mux[1]);
        val.at("gyro_mux_z").get_to(gyro_mux[2]);
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

}  // namespace kansei
