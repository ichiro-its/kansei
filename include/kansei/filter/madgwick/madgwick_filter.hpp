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

#ifndef KANSEI__FILTER__MADGWICK__MADGWICK_FILTER_HPP_
#define KANSEI__FILTER__MADGWICK__MADGWICK_FILTER_HPP_

#include "kansei/filter/madgwick.hpp"

#include "./iostream"
#include "./cmath"

namespace kansei
{

class MadgwickFilter
{
public:
  MadgwickFilter();
  virtual ~MadgwickFilter();

  void set_algorithm_gain(double gain)
  {
    gain_ = gain;
  }

  void set_drift_bias_gain(double zeta)
  {
    zeta_ = zeta;
  }

  void set_world_frame(WorldFrame frame)
  {
    world_frame_ = frame;
  }

  void get_orientation(double & q0, double & q1, double & q2, double & q3)
  {
    q0 = this->q0;
    q1 = this->q1;
    q2 = this->q2;
    q3 = this->q3;

    // perform precise normalization of the output, using 1/sqrt()
    // instead of the fast invSqrt() approximation. Without this,
    // TF2 complains that the quaternion is not normalized.
    double recipNorm = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
  }

  void set_orientation(double q0, double q1, double q2, double q3)
  {
    this->q0 = q0;
    this->q1 = q1;
    this->q2 = q2;
    this->q3 = q3;

    w_bx_ = 0;
    w_by_ = 0;
    w_bz_ = 0;
  }

  void madgwick_ahrs_update(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float mx, float my, float mz,
    float dt);

  void madgwick_ahrs_update_imu(
    float gx, float gy, float gz,
    float ax, float ay, float az,
    float dt);

  void get_gravity(
    float & rx, float & ry, float & rz,
    float gravity = 9.80665);

private:
  // paramaters
  double gain_;             // algorithm gain
  double zeta_;             // gyro drift bias gain
  WorldFrame world_frame_;  // NWU, ENU, NED

  // state variables
  double q0, q1, q2, q3;      // quaternion
  float w_bx_, w_by_, w_bz_;
};

}  // namespace kansei

#endif  // KANSEI__FILTER__MADGWICK__MADGWICK_FILTER_HPP_
