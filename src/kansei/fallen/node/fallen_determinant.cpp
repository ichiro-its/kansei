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
  std::string file_name = path + "kansei.json";
  std::ifstream file(file_name);

  if (!file.is_open()) {
    throw std::runtime_error("Failed to open file: " + file_name);
  }

  nlohmann::json imu_data = nlohmann::json::parse(file);

  for (const auto &[key, val] : imu_data.items()) {
    if (key == "accel_limit") {
      try {
        val.at("accel_back_limit").get_to(accel_back_limit);
        val.at("accel_front_limit").get_to(accel_front_limit);
        val.at("accel_right_limit").get_to(accel_right_limit);
        val.at("accel_left_limit").get_to(accel_left_limit);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    } else if (key == "orientation_limit") {
      try {
        val.at("pitch_back_limit").get_to(pitch_back_limit);
        val.at("pitch_front_limit").get_to(pitch_front_limit);
        val.at("roll_right_limit").get_to(roll_right_limit);
        val.at("roll_left_limit").get_to(roll_left_limit);
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
        throw ex;
      }
    }
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
