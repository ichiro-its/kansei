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

#include <fstream>
#include <memory>
#include <string>

#include "kansei/fallen/fallen.hpp"

#include "jitsuyo/config.hpp"
#include "keisan/keisan.hpp"
#include "nlohmann/json.hpp"

namespace kansei::fallen
{

FallenDeterminant::FallenDeterminant(const DeterminantType & type)
: fallen_status(FallenStatus::STANDUP), determinant_type(type),
  accel_back_limit(1000.0), accel_front_limit(0.0), accel_right_limit(1000.0), accel_left_limit(0.0),
  pitch_back_limit(100.0), pitch_front_limit(-100.0), roll_right_limit(-100.0), roll_left_limit(100.0)
{
}

void FallenDeterminant::load_config(const std::string & path)
{
  nlohmann::json imu_data;
  if (!jitsuyo::load_config(path, "kansei.json", imu_data)) {
    throw std::runtime_error("Failed to find config file `" + path + "kansei.json`");
  }

  bool valid_config = true;

  nlohmann::json accel_limit_section;
  if (jitsuyo::assign_val(imu_data, "accel_limit", accel_limit_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(accel_limit_section, "accel_back_limit", accel_back_limit);
    valid_section &= jitsuyo::assign_val(accel_limit_section, "accel_front_limit", accel_front_limit);
    valid_section &= jitsuyo::assign_val(accel_limit_section, "accel_right_limit", accel_right_limit);
    valid_section &= jitsuyo::assign_val(accel_limit_section, "accel_left_limit", accel_left_limit);
    if (!valid_section) {
      std::cout << "Error found at section `accel_limit`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  nlohmann::json orientation_limit_section;
  if (jitsuyo::assign_val(imu_data, "orientation_limit", orientation_limit_section)) {
    bool valid_section = true;
    valid_section &= jitsuyo::assign_val(orientation_limit_section, "pitch_back_limit", pitch_back_limit);
    valid_section &= jitsuyo::assign_val(orientation_limit_section, "pitch_front_limit", pitch_front_limit);
    valid_section &= jitsuyo::assign_val(orientation_limit_section, "roll_right_limit", roll_right_limit);
    valid_section &= jitsuyo::assign_val(orientation_limit_section, "roll_left_limit", roll_left_limit);
    if (!valid_section) {
      std::cout << "Error found at section `orientation_limit`" << std::endl;
      valid_config = false;
    }
  } else {
    valid_config = false;
  }

  if (!valid_config) {
    throw std::runtime_error("Failed to load config file `" + path + "kansei.json`");
  }
}

void FallenDeterminant::update_fallen_status(const keisan::Euler<double> & rpy)
{
  fallen_status = FallenStatus::STANDUP;

  if (rpy.pitch.degree() < pitch_front_limit) {
    fallen_status = FallenStatus::FORWARD;
  } else if (rpy.pitch.degree() > pitch_back_limit) {
    fallen_status = FallenStatus::BACKWARD;
  } else if (rpy.roll.degree() > roll_right_limit) {
    fallen_status = FallenStatus::RIGHT;
  } else if (rpy.roll.degree() < roll_left_limit) {
    fallen_status = FallenStatus::LEFT;
  }
}

void FallenDeterminant::update_fallen_status(const keisan::Vector<3> & acc)
{
  fallen_status = FallenStatus::STANDUP;

  if (acc[1] < accel_front_limit) {
    fallen_status = FallenStatus::FORWARD;
  } else if (acc[1] > accel_back_limit) {
    fallen_status = FallenStatus::BACKWARD;
  } else if (acc[0] < accel_right_limit) {
    fallen_status = FallenStatus::RIGHT;
  } else if (acc[0] > accel_left_limit) {
    fallen_status = FallenStatus::LEFT;
  }
}

bool FallenDeterminant::is_fallen() const
{
  return fallen_status != FallenStatus::STANDUP;
}

FallenStatus FallenDeterminant::get_fallen_status() const
{
  return fallen_status;
}

DeterminantType FallenDeterminant::get_determinant_type() const
{
  return determinant_type;
}

}  // namespace kansei::fallen
