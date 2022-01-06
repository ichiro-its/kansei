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

#include "kansei/measurement/node/measurement_unit.hpp"

#include "keisan/keisan.hpp"

using namespace keisan::literals;  // NOLINT

namespace kansei
{

MeasurementUnit::MeasurementUnit()
: rpy(0_deg, 0_deg, 0_deg), is_calibrated(false), acc_raw_rp(keisan::Vector<2>::zero())
{
}

keisan::Angle<double> MeasurementUnit::get_roll() const
{
  return rpy.roll;
}

keisan::Angle<double> MeasurementUnit::get_pitch() const
{
  return rpy.pitch;
}

keisan::Angle<double> MeasurementUnit::get_orientation() const
{
  return rpy.yaw.normalize();
}

keisan::Vector<2> MeasurementUnit::get_acc_rp() const
{
  return acc_raw_rp;
}

}  // namespace kansei
