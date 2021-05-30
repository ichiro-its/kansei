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

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <kansei/imu.hpp>
#include <kansei/stateless_orientation.hpp>

#include <iostream>
#include <string>
#include <memory>
#include <chrono>
#include <functional>

namespace kansei
{

Imu::Imu(std::string node_name)
  : rclcpp::Node(node_name) {

  initialized = false;
  filter.setWorldFrame(ENU);
  filter.setAlgorithmGain(0.1);
  filter.setDriftBiasGain(0.0);

  imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
    "imu/data_raw", rclcpp::QoS(rclcpp::SystemDefaultsQoS()),
    [this] (const sensor_msgs::msg::Imu::SharedPtr imu_msg_raw) {
      const geometry_msgs::msg::Vector3& ang_vel = imu_msg_raw->angular_velocity;
      const geometry_msgs::msg::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
      rclcpp::Time time = imu_msg_raw->header.stamp;

      if (!initialized) {
        geometry_msgs::msg::Quaternion init_q;
        if (!StatelessOrientation::computeOrientation(ENU, lin_acc, init_q)) {
          RCLCPP_WARN_STREAM(get_logger(), "The IMU seems to be in free fall, cannot determine gravity direction!");
          return;
        }

        RCLCPP_INFO(get_logger(), "First IMU message received.");
        filter.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);
        last_time = time;
        initialized = true;
      }

      last_time = time;
      filter.madgwickAHRSupdateIMU(
        ang_vel.x, ang_vel.y, ang_vel.z,
        lin_acc.x, lin_acc.y, lin_acc.z,
        (time - last_time).seconds());

      double roll = 0.0;
      double pitch = 0.0;
      double yaw = 0.0;
      double q0, q1, q2, q3;
      filter.getOrientation(q0, q1, q2, q3);
      tf2::Matrix3x3(tf2::Quaternion(q1,q2,q3,q0)).getRPY(roll, pitch, yaw);
      RCLCPP_INFO(get_logger(), "Roll %f, Pitch %f, Yaw %f", roll, pitch, yaw);
    }
  );
}

}  // namespace kansei
