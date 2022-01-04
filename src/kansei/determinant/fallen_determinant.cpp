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

#include "kansei/determinant/fallen_determinant.hpp"

#include "kansei/determinant/fallen_status.hpp"

namespace kansei
{

FallenDeterminant::FallenDeterminant()
: rpy(0_deg, 0_deg, 0_deg), fallen_status(FallenStatus::STANDUP),
  fallen_back_raw_limit(491.0), fallen_front_raw_limit(458.0),
  fallen_right_raw_limit(519.0), fallen_left_raw_limit(498.0)
{
}

void FallenDeterminant::update_fallen_status()
{
  fallen_status = FallenStatus::STANDUP;

  if (raw_acc_pitch < fallen_front_raw_limit) {
    fallen_status = FallenStatus::FORWARD;
  } else if (raw_acc_pitch > fallen_back_raw_limit) {
    fallen_status = FallenStatus::BACKWARD;
  } else if (raw_acc_roll > fallen_right_raw_limit) {
    fallen_status = FallenStatus::RIGHT;
  } else if (raw_acc_roll < fallen_left_raw_limit) {
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

}  // namespace kansei
