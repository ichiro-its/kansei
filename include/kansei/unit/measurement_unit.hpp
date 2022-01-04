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

#ifndef KANSEI__UNIT__MEASUREMENT_UNIT_HPP_
#define KANSEI__UNIT__MEASUREMENT_UNIT_HPP_

#include "keisan/keisan.hpp"

namespace kansei
{

class MeasurementUnit
{
public:
  MeasurementUnit();
  virtual ~MeasurementUnit() {}

  virtual void update_rpy() {}

  virtual void reset_orientation() {}
  virtual void set_orientation_to(const keisan::Angle<double> & target_orientation) {}

  keisan::Angle<double> get_roll() const;
  keisan::Angle<double> get_pitch() const;
  keisan::Angle<double> get_orientation() const;

  keisan::Euler<double> rpy;

  bool is_calibrated;
};

}  // namespace kansei

#endif  // KANSEI__UNIT__MEASUREMENT_UNIT_HPP_
